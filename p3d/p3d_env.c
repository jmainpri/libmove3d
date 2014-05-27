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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#include "env.hpp"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

#define sqr(x) ((x) * (x))

#ifdef LIGHT_PLANNER
#include "ManipulationArmData.hpp"
#endif

extern double ZminEnv;
extern double ZmaxEnv;

/************************************************************************/

pp3d_env  XYZ_ENV   = NULL;
pp3d_env  *XYZ_TAB_ENV  = NULL;
int       XYZ_NUM_ENV  = 0;
int       XYZ_MAX_NUM_ENV = 0; // use P3D_MAX_ENV as block size for realloc

pp3d_rob  XYZ_ROBOT;
pp3d_obj  XYZ_OBSTACLES;
pp3d_traj  XYZ_TRAJS;        /*    is used but in this file, serves    *
                              *    as a sort of "current trajectory"   */

pp3d_graph XYZ_GRAPH = NULL;

static pp3d_traj *XYZ_TRAJ = NULL;       /* Carl 28032001 */
static int nof_trajs = 0;                /* Carl 28032001 */

/************************************************************************/

/*extern char *strdup(const char *);*/

static int       E_DEF, O_DEF, R_DEF, T_DEF;
static int       INIT = FALSE;

/************************************************************************/

//int p3d_end_desc(void);
static int p3d_init(void);
/* static */
void *p3d_beg_env(char* name);
static int p3d_end_env(void);
/* static */
void *p3d_beg_obj(char *name, int type);
int p3d_end_obj(void);
static void *p3d_beg_rob(char* name);
static int p3d_end_rob(void);
static void *p3d_beg_traj(char* name);
static int p3d_end_traj(void);

/* static */
void move_point(p3d_matrix4 pos, double *x, double *y, double *z, int point);
#ifdef MULTIGRAPH
static p3d_multiGraphJoint * p3d_cloneMultiGraphJoint(p3d_multiGraphJoint * src);
#endif

//extern int p3d_polynum;


/*
 * function that computes the array of lengths for the distance
 * computations
 *
 * some distance functions (like linear and Manhattan) are computed
 * sum values of the joints. These values can represent either a length
 * if the joint work in translation or an angle if the joint is rotoid.
 * In this latter case, the value of the joint has to be multiplied by a
 * length to make it homogeneous with a length.
 */
/* modif. Carl UI KINEMATICS: pass on pointer to robot rather than
   using "current robot" stored in global variable XYZ_ROBOT */
void compute_length_array(pp3d_rob robotPt) {
  /* pp3d_rob robotPt = XYZ_ROBOT; */
  pp3d_jnt jointPt;
  int i, j, njnt = robotPt->njoints;

  robotPt->length_array = MY_ALLOC(double, njnt + 1);

  for (i = 0; i <= robotPt->njoints; i++) {
    jointPt = robotPt->joints[i];
    robotPt->length_array[i] = 1.0;
    for (j = 0; j < jointPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jointPt, j)) {
        robotPt->length_array[i] = jointPt->dist;
      }
    }
  }
}


/***************************************************/
/* Fonction commencant une description             */
/* In : le typre de la description, son nom        */
/* Out :                                           */
/***************************************************/
void *p3d_beg_desc(int type, char *name) {
  void *desc = NULL;

  switch (type) {
    case P3D_ENV:
      if (XYZ_NUM_ENV == P3D_MAX_ENV - 1) {
        PrintError(("MP: p3d_beg_desc: too many ENV...\n"));
        return(NULL);
      }
      if (E_DEF) {
        PrintError(("MP: p3d_beg_desc: already in DEF_ENV mode\n"));
        return(NULL);
      }
      desc = p3d_beg_env(name);
      if (!desc) {
        PrintError(("MP: p3d_beg_desc: can't create a new ENV\n"));
      } else {
        E_DEF = TRUE;
      }
      break;

    case P3D_ROBOT:
      if (!E_DEF) {
        PrintError(("MP: p3d_beg_desc: not in DEF_ENV mode\n"));
        return(NULL);
      }
      if (R_DEF) {
        PrintWarning(("MP: p3d_beg_desc: I closed previous ROBOT\n"));
        if (O_DEF) p3d_end_desc();
        p3d_end_desc();
      }
      if (O_DEF) {
        PrintWarning(("MP: p3d_beg_desc: I closed previous OBSTACLE\n"));
        p3d_end_desc();
      }
      desc = p3d_beg_rob(name);
      if (!desc) {
        PrintError(("MP: p3d_beg_desc: can't create a new ROBOT\n"));
      } else {
        R_DEF = TRUE;
      }
      break;

    case P3D_OBSTACLE:
      if (!E_DEF) {
        PrintError(("MP: p3d_beg_desc: not in DEF_ENV mode\n"));
        return(NULL);
      }
      if (R_DEF) {
        PrintError(("MP: p3d_beg_desc: already in DEF_ROBOT mode\n"));
        return(NULL);
      }
      if (O_DEF) {
        PrintWarning(("MP: p3d_beg_desc: I closed previous OBSTACLE\n"));
        p3d_end_desc();
      }
      desc = p3d_beg_obj(name, P3D_OBSTACLE);
      if (!desc) {
        PrintError(("MP: p3d_beg_desc: can't create a new OBSTACLE\n"));
      } else {
        O_DEF = TRUE;
      }
      break;

    case P3D_BODY:
      if (!R_DEF) {
        PrintError(("MP: p3d_beg_desc: not  in DEF_ROBOT mode\n"));
        return(NULL);
      }
      if (O_DEF) {
        PrintWarning(("MP: p3d_beg_desc: I closed previous BODY\n"));
        p3d_end_desc();
      }
      desc = p3d_beg_obj(name, P3D_BODY);
      if (!desc) {
        PrintError(("MP: p3d_beg_desc: can't create a new BODY\n"));
      } else      O_DEF = TRUE;
      break;

    case P3D_TRAJ:
      if (E_DEF || !XYZ_NUM_ENV) {
        PrintError(("MP: p3d_beg_desc: no ENV created\n"));
        return(NULL);
      }
      if (T_DEF) {
        PrintWarning(("MP: p3d_beg_desc: I closed previous TRAJ\n"));
        p3d_end_desc();
      }
      desc = p3d_beg_traj(name);
      if (!desc) {
        PrintError(("MP: p3d_beg_desc: can't create a new TRAJ\n"));
      } else      T_DEF = TRUE;
      break;
  }
  return(desc);
}

/**************************************/
/* Fonction terminant une description */
/* In :                               */
/* Out :                              */
/**************************************/
int p3d_end_desc(void) {
  if (O_DEF) {
    O_DEF = FALSE;
    return(p3d_end_obj());
  }
  if (R_DEF) {
    R_DEF = FALSE;
    return(p3d_end_rob());
  }
  if (E_DEF) {
    E_DEF = FALSE;
    if (XYZ_NUM_ENV + 1 > XYZ_MAX_NUM_ENV) {
      XYZ_MAX_NUM_ENV += P3D_MAX_ENV;
      XYZ_TAB_ENV = MY_REALLOC(XYZ_TAB_ENV, pp3d_env, (XYZ_MAX_NUM_ENV - P3D_MAX_ENV), XYZ_MAX_NUM_ENV);
    }
    XYZ_TAB_ENV[XYZ_NUM_ENV++] = (pp3d_env)(XYZ_ENV) ;
    p3d_end_env();

    return 0;
  }
  if (T_DEF) {
    T_DEF = FALSE;
    return(p3d_end_traj());
  }
  PrintWarning(("MP: p3d_end_desc: nothing to close!\n"));

  return(FALSE);
}

/******************************************************/
/* Fonction testant si on est en cours de description */
/* In :                                               */
/* Out :                                              */
/******************************************************/
int p3d_inside_desc(void) {
  return((E_DEF || T_DEF) ? TRUE : FALSE);
}



/*--------------------------------------------------------------------------*/
/*!
 * \brief This function create a joint.
 *
 * This joint is put at the position given by \a pos.
 * For each degree of freedom there is:
 *  - Its initial position: \a dtab[i_dof*3]
 *  - Its bounds: \a dtab[i_dof*3+1] and \a dtab[i_dof*3+2]
 *  - Its random (or user) bounds: \a dtab2[i_dof*2] and \a dtab2[i_dof*2+1]
 * There is also:
 *  - The parameters of the joint (the value of the array \a dtab
 *                                 after the indice 3*nb_dof)
 *  - The scale factor: \a scale
 *  - The previous joint: \a prev
 * - Its velocity and torque maximal values: \a dtab3[i_dof*2] and \a dtab3[i_dof*2+1]
 *
 * Note: this function update the previous joint structure and
 *       the robot structure.
 *
 * \param  type:  The type of joint
 * \param  pos:   The position matrix to place the joint
 * \param  dtab:  Array that holds parameters for the joint (see behind)
 * \param  prev:  The indice of the previous joint
 * \param  dtab2: Array that holds parameters for the joint (see behind)
 * \param  scale: Scale factor
 *
 * \retval the nuber of joints in the robot, FALSE otherwise
 */
int p3d_add_desc_jnt_deg(p3d_type_joint type, p3d_matrix4 pos,  double * dtab,
                         int prev, double * dtab2, double scale, double *dtab3, double *vel_max, double *acc_max, double *jerk_max) {
  pp3d_jnt jnt, prev_jnt, *newj;
  int      i, n, nb_dof, nb_param;
  char name[JNT_MAX_SIZE_NAME];
  double V[JNT_NB_DOF_MAX], Vmin[JNT_NB_DOF_MAX], Vmax[JNT_NB_DOF_MAX],
  Vmin_rand[JNT_NB_DOF_MAX], Vmax_rand[JNT_NB_DOF_MAX];
  double Velocity_max[JNT_NB_DOF_MAX], Torque_max[JNT_NB_DOF_MAX];
  double Acceleration_max[JNT_NB_DOF_MAX],  Jerk_max[JNT_NB_DOF_MAX];

  if (!R_DEF) {
    PrintError(("MP: p3d_add_desc_jnt_deg: not DEF_ROB mode\n"));
    return(FALSE);
  }

  if (O_DEF) {
    PrintWarning(("MP: p3d_add_desc_jnt_deg: I closed previous BODY\n"));
    p3d_end_desc();
  }

  if (XYZ_ROBOT->njoints != -1) {
    if ((prev < -1) || (prev > XYZ_ROBOT->njoints)) {
      PrintError(("MP: p3d_add_desc_jnt_deg: incorrect id (= %i) for prev_jnt\n", prev));
      return(FALSE);
    }
  }

  p3d_jnt_get_nb_param(type, &nb_dof, &nb_param);
  for (i = 0; i < nb_dof; i++) {
    if (dtab[i*3+1] <= dtab[i*3+2]) {
      Vmin[i]    = dtab[i*3+1];
      Vmax[i]    = dtab[i*3+2];
    } else {
      Vmax[i]    = dtab[i*3+1];
      Vmin[i]    = dtab[i*3+2];
    }
    V[i]         = MAX(Vmin[i], MIN(Vmax[i], dtab[i*3]));
    Vmin_rand[i] = MAX(Vmin[i], MIN(Vmax[i], dtab2[i*2]));
    Vmax_rand[i] = MAX(Vmin_rand[i], MIN(Vmax[i], dtab2[i*2+1]));
    //Velocity_max[i]= dtab3[i*2];
    Torque_max[i]  = dtab3[i*2+1];
    Velocity_max[i]= vel_max[i];
    Acceleration_max[i]= acc_max[i];
    Jerk_max[i]= jerk_max[i];
  }
  /* compatibility */
  for (i = nb_dof; i < JNT_NB_DOF_MAX; i++) {
    V[i]         = V[nb_dof-1];
    Vmin[i]      = Vmin[nb_dof-1];
    Vmax[i]      = Vmax[nb_dof-1];
    Vmin_rand[i] = Vmin_rand[nb_dof-1];
    Vmax_rand[i] = Vmax_rand[nb_dof-1];
    Velocity_max[i] = Velocity_max[nb_dof-1];
    Torque_max[i]   = Torque_max[nb_dof-1];
  }


  if ((XYZ_ROBOT->njoints != -1) && (prev >= 0)) {
    prev_jnt      = XYZ_ROBOT->joints[prev];
  } else {
    prev_jnt = NULL;
  }
  if (XYZ_ROBOT->njoints == -1) {
    type = P3D_BASE;  /* compatibility */
  }

  jnt = p3d_jnt_create_deg(type, pos, V, Vmin, Vmax, Vmin_rand,
                           Vmax_rand, Velocity_max, Acceleration_max, Jerk_max, Torque_max, dtab + 3 * nb_dof);
  if (!jnt) {
    PrintWarning(("MP: p3d_add_desc_jnt_deg: can't create a new joint\n"));
    return(FALSE);
  }
  p3d_jnt_scale(jnt, scale);
  p3d_jnt_attach_to_jnt(prev_jnt, jnt);


  /* on actualise le tableau des joints du robot*/
  jnt->num = n = ++XYZ_ROBOT->njoints;
  jnt->index_dof = XYZ_ROBOT->nb_dof;
  jnt->index_user_dof = XYZ_ROBOT->nb_user_dof;
  XYZ_ROBOT->nb_dof += jnt->dof_equiv_nbr;
  XYZ_ROBOT->nb_user_dof += jnt->user_dof_equiv_nbr;
  jnt->rob = XYZ_ROBOT;
  newj     = MY_ALLOC(p3d_jnt *, n + 1);
  if (newj) {
    for (i = 0;i < n;i++) {
      newj[i] = XYZ_ROBOT->joints[i];
    }
    newj[n] = jnt;
    if (n > 0) {
      MY_FREE(XYZ_ROBOT->joints, p3d_jnt *, n);
    }
    XYZ_ROBOT->joints = newj;
  }

  sprintf(name, "J%i", jnt->num);
  p3d_jnt_set_name(jnt, name);

  // initialization of flag is_active_for_planner
  // by default = TRUE
  p3d_jnt_set_is_active_for_planner(jnt, TRUE);

  return(XYZ_ROBOT->njoints);
}

/******************************************************/
/* Fonction commencant la description du polyhedre de */
/* l'objet courant                                    */
/* In : le nom du polyhedre                           */
/* Out :                                              */
/******************************************************/
void p3d_add_desc_poly(char name[20], int type) {
  XYZ_OBSTACLES->polcur = p3d_poly_beg_poly(name, type);
}

/*******************************************************/
/* Fonction ajoutant un sommet au polyhedre de l'objet */
/* courant                                             */
/* In : les coordonnees du sommet                      */
/* Out :                                               */
/*******************************************************/
void p3d_add_desc_vert(double x, double y, double z) {
  p3d_poly_add_vert(XYZ_OBSTACLES->polcur, x, y, z);
}


/*******************************************************/
/* Fonction ajoutant  une face au polyhedre de l'objet */
/* courant                                             */
/* In : la liste des sommets, le nombre de sommets     */
/* Out :                                               */
/*******************************************************/
void p3d_add_desc_face(int *listeV, int nb_Vert) {
  p3d_poly_add_face(XYZ_OBSTACLES->polcur, listeV, nb_Vert);
}

/***************************************************/
/* Foncton terminant la description du polyhedre   */
/* de l'objet courant                              */
/* In :                                            */
/* Out :                                           */
/***************************************************/
void p3d_end_desc_poly(void) {
  int np = 0, i;
  p3d_poly **newpol = NULL, **oldpol = NULL;

  p3d_poly_end_poly(XYZ_OBSTACLES->polcur);

  np = XYZ_OBSTACLES->np;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++) {
      newpol[i] = oldpol[i];
    }
    newpol[np] = XYZ_OBSTACLES->polcur;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = XYZ_OBSTACLES->polcur;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np = XYZ_OBSTACLES->np + 1;
}

/**********************************************/
/* Fonction commencant la description du cube */
/* de l'objet courant                         */
/* In : le nom du cube, son cote              */
/* Out :                                      */
/**********************************************/
void p3d_add_desc_cube(char name[20], double a, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  a= (double) fabs(a);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_cube(name, a, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }

  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description de la boite */
/* de l'objet courant                             */
/* In : le nom de la boite, ses cotes             */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_box(char name[20], double a, double b, double c, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  a= (double) fabs(a);
  b= (double) fabs(b);
  c= (double) fabs(c);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_box(name, a, b, c, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
  XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}
/**************************************************/
/* Fonction commencant la description de la boite */
/* (swept rectangle) de l'objet courant           */
/* In : le nom de la boite, ses cotes             */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_srect(char name[20], double a, double b, double h,
                        double ux, double uy, double uz,
                        double vx, double vy, double vz, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  a= (double) fabs(a);
  b= (double) fabs(b);
  h= (double) fabs(h);
  ux= (double) fabs(ux);
  uy= (double) fabs(uy);
  uz= (double) fabs(uz);
  vx= (double) fabs(vx);
  vy= (double) fabs(vy);
  vz= (double) fabs(vz);


  np = XYZ_OBSTACLES->np;

  p = p3d_create_srect(name, a, b, h, ux, uy, uz, vx, vy, vz, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
  XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description de la       */
/* pyramide de l'objet courant                    */
/* In : le nom de la boite, ses cotes             */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_pyramid(char name[20], double a, double b, double c,
                          double d, double e, double f, double g, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  a= (double) fabs(a);
  b= (double) fabs(b);
  c= (double) fabs(c);
  d= (double) fabs(d);
  e= (double) fabs(e);
  f= (double) fabs(f);
  g= (double) fabs(g);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_pyramid(name, a, b, c, d, e, f, g, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du cylindre */
/* de l'objet courant                             */
/* In : le nom du cylindre, son rayon, son cote   */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_cylindre(char name[20], double r, double l, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  r= (double) fabs(r);
  l= (double) fabs(l);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_cylindre(name, r, l, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du cylindre */
/* oval de l'objet courant                        */
/* In : le nom du cylindre, son rayon, son cote   */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_cylindre_oval(char name[20], double a, double b, double l, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  a= (double) fabs(a);
  b= (double) fabs(b);
  l= (double) fabs(l);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_cylindre_oval(name, a, b, l, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}


/**************************************************/
/* Fonction commencant la description du prisme   */
/* de l'objet courant                             */
/* In : le nom du prisme, son nombre de faces,    */
/* son rayon, son cote                            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_prisme(char name[20], int nvert, double r, double l, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  r= (double) fabs(r);
  l= (double) fabs(l);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_prisme(name, nvert, r, l, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }
XYZ_OBSTACLES->pol[np]->forceBBComputation = FALSE;
  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du cone     */
/* de l'objet courant                             */
/* In : le nom du prisme, son nombre de faces,    */
/* son rayon, son cote                            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_cone(char name[20], int nvert, double r1, double r2, double l, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  r1= (double) fabs(r1);
  r2= (double) fabs(r2);
  l= (double) fabs(l);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_cone(name, nvert, r1, r2, l, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }

  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}


/**************************************************/
/* Fonction commencant la description du snout    */
/* de l'objet courant                             */
/* In : le nom du prisme, son nombre de faces,    */
/* son rayon, son cote                            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_snout(char name[20], int nvert, double dt, double db,
                        double height, double xoff, double yoff, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_snout(name, nvert, dt, db, height, xoff, yoff, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }

  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du snout    */
/* de l'objet courant                             */
/* In : le nom du prisme, son nombre de faces,    */
/* son rayon, son cote                            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_skew_snout(char name[20], int nvert, double dt, double db,
                             double height, double xoff, double yoff, double a1,
                             double a2, double a3, double a4, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_skew_snout(name, nvert, dt, db, height, xoff, yoff, a1, a2, a3, a4, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }

  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description de la sphere*/
/* de l'objet courant                             */
/* In : le nom de la sphere, son rayon            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_rtorusslice(char name[20], double r1, double r2,
                              double height, double angle, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_rtorusslice(name, r1, r2, height, angle, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du    tore  */
/* spherique   de l'objet courant                 */
/* In : le nom de la sphere, son rayon            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_ctorusslice(char name[20], double R, double radius,
                              double angle, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_ctorusslice(name, R, radius, angle, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du     tore */
/* rectangulaire de l'objet courant               */
/* In : le nom de la sphere, son rayon            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_sphere(char name[20], double r, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  r= (double) fabs(r);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_sphere(name, r, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description d'un demi   */
/* sphere de l'objet courant                      */
/* In : le nom de la sphere, son rayon et hauteur */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_half_sphere(char name[20], double r, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_half_sphere(name, r, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}


/**************************************************/
/* Fonction commencant la description d'un demi   */
/* review sphere de l'objet courant               */
/* In : le nom de la sphere, son rayon et hauteur */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_sphere_shell(char name[20], double r, double h, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_sphere_shell(name, r, h, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description du   Review */
/* swept annulus de l'objet courant               */
/* In : le nom de la sphere, ses parametres       */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_sweptrectslice(char name[20], double r1, double r2,
                                 double l, double a, double ux, double uy,
                                 double uz, double vx, double vy, double vz, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_sweptrectslice(name, r1, r2, l, a, ux, uy, uz, vx, vy, vz, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description de la sphere*/
/* de l'objet courant                             */
/* In : le nom de la sphere, son rayon            */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_oval(char name[20], double a, double b, double c, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_oval(name, a, b, c, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/**************************************************/
/* Fonction commencant la description d'un demi   */
/* ellipsoide de l'objet courant                  */
/* In : le nom de la sphere, ses rayons           */
/* Out :                                          */
/**************************************************/
void p3d_add_desc_half_oval(char name[20], double a, double b, double c, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;

  np = XYZ_OBSTACLES->np;

  p = p3d_create_half_oval(name, a, b, c, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}

/*****************************************************/
/* Fonction commencant la description de la portion  */
/* de tore de l'objet courant                        */
/* In : le nom de la sphere, son rayon, son angle    */
/* Out :                                             */
/*****************************************************/
void p3d_add_desc_tore(char name[20], double r, double a, int type) {
  int np, i;
  p3d_poly *p = NULL, **newpol = NULL, **oldpol = NULL;
  double ar = DTOR(a);

  np = XYZ_OBSTACLES->np;

  p = p3d_create_tore(name, r, r, ar, type);

  XYZ_OBSTACLES->polcur = p;

  if (np > 0) {
    oldpol = XYZ_OBSTACLES->pol;
    newpol = MY_ALLOC(p3d_poly *, np + 1);
    for (i = 0;i < np;i++)  newpol[i] = oldpol[i];
    newpol[np] = p;
    XYZ_OBSTACLES->pol = newpol;
    MY_FREE(oldpol, p3d_poly *, np);
  } else {
    XYZ_OBSTACLES->pol = MY_ALLOC(p3d_poly *, 1);
    XYZ_OBSTACLES->pol[0] = p;
  }


  /* Boite de l'objet courant.... */
  XYZ_OBSTACLES->box.x1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x1, XYZ_OBSTACLES->box.x1);
  XYZ_OBSTACLES->box.x2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.x2, XYZ_OBSTACLES->box.x2);
  XYZ_OBSTACLES->box.y1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y1, XYZ_OBSTACLES->box.y1);
  XYZ_OBSTACLES->box.y2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.y2, XYZ_OBSTACLES->box.y2);
  XYZ_OBSTACLES->box.z1 = MIN(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z1, XYZ_OBSTACLES->box.z1);
  XYZ_OBSTACLES->box.z2 = MAX(XYZ_OBSTACLES->pol[XYZ_OBSTACLES->np]->box.z2, XYZ_OBSTACLES->box.z2);

  XYZ_OBSTACLES->np =  XYZ_OBSTACLES->np + 1;
}



/*
 *  Add a local path to the current trajectory
 *
 *  Input:  the new local path
 *
 */
int p3d_add_desc_courbe(p3d_localpath *localpathPt) {
  int       numloc;
  p3d_localpath *last_localpathPt = XYZ_TRAJS->courbePt;


  XYZ_TRAJS->nlp = XYZ_TRAJS->nlp + 1;
  numloc = XYZ_TRAJS->nlp;

  localpathPt->lp_id = numloc;

  if (last_localpathPt == NULL) {
    last_localpathPt = localpathPt;
    XYZ_TRAJS->courbePt = last_localpathPt;
  } else {
    /* add the local path at the end of the list */
    while (last_localpathPt->next_lp != NULL) {
      last_localpathPt = last_localpathPt->next_lp;
    }
    last_localpathPt->next_lp = localpathPt;
    localpathPt->prev_lp = last_localpathPt;
  }

  return(XYZ_TRAJS->nlp);
}

/**************************************************/
/* Fonction changeant la couleur d'un obstacle    */
/* In : le nom de l'obstacle, sa couleur          */
/* Out :                                          */
/**************************************************/

void p3d_set_obst_color(char *name, int color, double *color_vect) {
  p3d_obj *obj;
  int i;

  obj = p3d_get_obst_by_name(name);
  for (i = 0;i < obj->np;i++) {
    p3d_poly_set_color(obj->pol[i], color, color_vect);
  }
}

/**
 * Computes the color gradiant
 * color: the output vector 
 * x: the gradiant beetween 0 (Green) and 1 (Red)
 * min and max: variation of the RGB channels (Move3D 0 -> 1)
 */
void GroundColorMixGreenToRed(double* color, double x)
{
	if (x>1) {
		x = 1;
	}
	if (x<0) {
		x=0;
	}
	GroundColorMix(color, 180*(1 - x), 0, 1);
}

/**
 * Computes the color gradiant
 * color: the output vector 
 * x: the gradiant (beetween 0 and 360)
 * min and max: variation of the RGB channels (Move3D 0 -> 1)
 */
void GroundColorMix(double* color, double x, double min, double max)
{
	/*
	 * Red = 0
	 * Green = 1
	 * Blue = 2
	 */
	double posSlope = (max-min)/60;
	double negSlope = (min-max)/60;

	if( x < 60 )
	{
		color[0] = max;
		color[1] = posSlope*x+min;
		color[2] = min;
		return;
	}
	else if ( x < 120 )
	{
		color[0] = negSlope*x+2*max+min;
		color[1] = max;
		color[2] = min;
		return;
	}
	else if ( x < 180  )
	{
		color[0] = min;
		color[1] = max;
		color[2] = posSlope*x-2*max+min;
		return;
	}
	else if ( x < 240  )
	{
		color[0] = min;
		color[1] = negSlope*x+4*max+min;
		color[2] = max;
		return;
	}
	else if ( x < 300  )
	{
		color[0] = posSlope*x-4*max+min;
		color[1] = min;
		color[2] = max;
		return;
	}
	else
	{
		color[0] = max;
		color[1] = min;
		color[2] = negSlope*x+6*max;
		return;
	}
}

/**************************************************/
/* Fonction changeant la couleur d'un polyhedre   */
/* d'un obstacle                                  */
/* In : le nom de l'obstacle, sa couleur          */
/* Out :                                          */
/**************************************************/
void p3d_set_obst_poly_color(char *name, int num, int color,
                             double *color_vect) {
  p3d_obj *obj;
  double zAverage, z1, z2, z3, colorCoefficient;

  obj = p3d_get_obst_by_name(name);
  
  if (ENV.getBool(Env::isCostSpace) == FALSE) {
    p3d_poly_set_color(obj->pol[num-1], color, color_vect);
  } else {

    z1 = (obj->pol[num-1])->poly->the_points[0][2];
    z2 = (obj->pol[num-1])->poly->the_points[1][2];
    z3 = (obj->pol[num-1])->poly->the_points[2][2];

    zAverage = (z1 + z2 + z3) / 3.;
#ifdef P3D_PLANNER
    colorCoefficient = zAverage / (ZmaxEnv - ZminEnv);
#endif

    GroundColorMix(color_vect, 280*(1-colorCoefficient), 0, 1);

//    printf("color Coefficient = %f\n",colorCoefficient);
//    printf("color_vector[%d] = %f\n",0,color_vect[0]);
//    printf("color_vector[%d] = %f\n",1,color_vect[1]);
//    printf("color_vector[%d] = %f\n",2,color_vect[2]);
//    printf("\n");

//    color_vect[0] = colorCoefficient;
//    color_vect[1] = 0;
//    color_vect[2] = 1 - colorCoefficient;GroundColorMix
    color_vect[3] = 1;

    p3d_poly_set_color(obj->pol[num-1], color, color_vect);
    //  }
  }
}


/******************************************************/
/* Fonction changeant la couleur d'un corps de robot  */
/* In : le nom de l'obstacle, sa couleur              */
/* Out :                                              */
/******************************************************/

void p3d_set_body_color(char *name, int color, double *color_vect) {
  p3d_obj *obj;
  int i;

  obj = p3d_get_body_by_name(name);
  if (obj == NULL) {
    PrintError(("No body with name %s declared!\n", name));
  } else {
    for (i = 0;i < obj->np;i++) {
      p3d_poly_set_color(obj->pol[i], color, color_vect);
    }
  }
}

/******************************************************/
/* Fonction changeant la couleur d'un polyhedre d'un  */
/* corps de robot                                     */
/* In : le nom de l'obstacle, sa couleur              */
/* Out :                                              */
/******************************************************/
void p3d_set_body_poly_color(char *name, int num, int color, double *color_vect) {
  p3d_obj *obj;

  obj = p3d_get_body_by_name(name);
  if (obj == NULL) {
    PrintError(("No body with name %s declared!\n", name));
  } else {
    p3d_poly_set_color(obj->pol[num-1], color, color_vect);
  }
}



/******************************************************/
/* Fonction selectionannt la description courante par */
/* son nom                                            */
/* In : le type a selectionner, son nom               */
/* Out :                                              */
/******************************************************/
void *p3d_sel_desc_name(int type, char* name) {
  int i;

  switch (type) {
    case P3D_ENV:
      for (i = 0;i < XYZ_NUM_ENV;i++) {
        if (XYZ_TAB_ENV[i]) {
          if (strcmp(XYZ_TAB_ENV[i]->name, name) == 0) {
            XYZ_ENV = XYZ_TAB_ENV[i];
#ifdef P3D_PLANNER
            if (XYZ_ENV->cur_robot) // Modification Fabien
              XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;
#endif
			  
            return((void *)XYZ_ENV);
          }
        }
      }
      PrintError(("MP: p3d_sel_desc_name: wrong name (%s)\n", name));
      return(NULL);

    case P3D_OBSTACLE:
      for (i = 0;i < XYZ_ENV->no;i++)
        if (strcmp(XYZ_ENV->o[i]->name, name) == 0) {
          XYZ_ENV->ocur = XYZ_ENV->o[i];
          return((void *)(XYZ_ENV->ocur));
        }
      PrintError(("MP: p3d_sel_desc_name: wrong name (%s)\n", name));
      return(NULL);

    case P3D_ROBOT:
      for (i = 0;i < XYZ_ENV->nr;i++)
        if (strcmp(XYZ_ENV->robot[i]->name, name) == 0) {
          XYZ_ENV->cur_robot = XYZ_ENV->robot[i];
#ifdef P3D_PLANNER
          XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;  // Modification Fabien
          if (XYZ_GRAPH != NULL) {
            p3d_set_ORIENTED(XYZ_GRAPH->oriented);
          }
#endif
          return((void *)(XYZ_ENV->cur_robot));
        }
      PrintError(("MP: p3d_sel_desc_name: wrong name (%s)\n",name));
      return(NULL);
      /* break; */

    case P3D_BODY:
      if (XYZ_ENV->cur_robot != NULL) {
        for (i = 0; i < XYZ_ENV->cur_robot->no;i++) {
          if (strcmp(XYZ_ENV->cur_robot->o[i]->name, name) == 0) {
            XYZ_ENV->cur_robot->ocur = XYZ_ENV->cur_robot->o[i];
            return((void *)(XYZ_ENV->cur_robot->ocur));
          }
        }
      }
      PrintError(("MP: p3d_sel_desc_name: wrong name (%s)\n", name));
      return(NULL);

    case P3D_TRAJ:
      if (XYZ_ENV->cur_robot != NULL) {
        for (i = 0;i < XYZ_ENV->cur_robot->nt;i++) {
          if (strcmp(XYZ_ENV->cur_robot->t[i]->name, name) == 0) {
            XYZ_ENV->cur_robot->tcur = XYZ_ENV->cur_robot->t[i];
            return((void *)(XYZ_ENV->cur_robot->tcur));
          }
        }
      }
      PrintError(("MP: p3d_sel_desc_name: wrong name (%s)\n", name));
      return(NULL);

    default:
      PrintError(("MP: p3d_sel_desc_name: wrong type\n"));
      return(NULL);
  }
}

/******************************************************/
/* Fonction selectionannt la description courante par */
/* son numero                                         */
/* In : le type a selectionner, son numero            */
/* Out :                                              */
/******************************************************/
void *p3d_sel_desc_num(int type, int num) {

  if (type == P3D_NULL_OBJ) return(NULL);
  switch (type) {
    case P3D_ENV:
      if ((num < 0) || (num >= XYZ_NUM_ENV)) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (env): %d\n", num));
        return(NULL);
      }
      if (XYZ_TAB_ENV[num] == NULL) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (env): %d\n", num));
        return(NULL);
      }
      XYZ_ENV = XYZ_TAB_ENV[num];
#ifdef P3D_PLANNER
      if (XYZ_ENV->cur_robot) // Modification Fabien
        XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;
#endif
      return((void *)XYZ_ENV);

    case P3D_OBSTACLE:
      if ((num < 0) || (num >= XYZ_ENV->no)) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (obst): %d\n", num));
        return(NULL);
      }
      XYZ_ENV->ocur = XYZ_ENV->o[num];
      return((void *)XYZ_ENV->ocur);

    case P3D_ROBOT:
      if ((num < 0) || (num >= XYZ_ENV->nr)) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (rob)\n"));
        return(NULL);
      }
      XYZ_ENV->cur_robot = XYZ_ENV->robot[num];
#ifdef P3D_PLANNER
      XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;  // Modification Fabien
      XYZ_ROBOT = XYZ_ENV->cur_robot;
      if (XYZ_GRAPH != NULL) {
        p3d_set_ORIENTED(XYZ_GRAPH->oriented);
      }
#endif
      return((void *)XYZ_ENV->cur_robot);

    case P3D_BODY:
      if ((num < 0) || (num >= XYZ_ENV->cur_robot->no)) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (body)\n"));
        return(NULL);
      }
      XYZ_ENV->cur_robot->ocur = XYZ_ENV->cur_robot->o[num];
      return((void *)XYZ_ENV->cur_robot->ocur);

    case P3D_TRAJ:
      if (XYZ_ENV == NULL) return(NULL);
      if (XYZ_ENV->cur_robot == NULL) return(NULL);
      if ((num < 0) || (num >= XYZ_ENV->cur_robot->nt)) {
        PrintError(("MP: p3d_sel_desc_num: wrong num (traj)\n"));
        return(NULL);
      }
      XYZ_ENV->cur_robot->tcur = XYZ_ENV->cur_robot->t[num];
      return((void *)XYZ_ENV->cur_robot->tcur);

    case TRJ_TRAJ:
      if (XYZ_TRAJ == NULL) return(NULL);
      if (num >= nof_trajs) return(NULL);
      if (num < 0) return(NULL);
      return((void *)XYZ_TRAJ[num]);
    default:
      PrintError(("MP: p3d_sel_desc_num: wrong type\n"));
      return(NULL);
  }
}

/******************************************************/
/* Fonction selectionannt la description courante par */
/* son id                                             */
/* In : le type a selectionner, son id                */
/* Out :                                              */
/******************************************************/
void *p3d_sel_desc_id(int type, void *id) {
  int i;

  if (!id) {
    PrintError(("MP: p3d_sel_desc_id: wrong identifier\n"));
    return(NULL);
  }

  switch (type) {

    case P3D_ENV:
      for (i = 0;i < XYZ_NUM_ENV;i++)
        if (XYZ_TAB_ENV[i] == (pp3d_env)id) {
          XYZ_ENV = XYZ_TAB_ENV[i];
#ifdef P3D_PLANNER
          if (XYZ_ENV->cur_robot)
            XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;  // Modification Fabien
#endif
          return((void *)XYZ_ENV);
        }
      PrintError(("MP: p3d_sel_desc_id: wrong id\n"));
      return(NULL);

    case P3D_OBSTACLE:
      for (i = 0;i < XYZ_ENV->no;i++)
        if (XYZ_ENV->o[i] == (pp3d_obj)id) {
          XYZ_ENV->ocur = XYZ_ENV->o[i];
          return((void *)(XYZ_ENV->ocur));
        }
      PrintError(("MP: p3d_sel_desc_id: wrong id\n"));
      return(NULL);

    case P3D_ROBOT:
      for (i = 0;i < XYZ_ENV->nr;i++)
        if (XYZ_ENV->robot[i] == (pp3d_rob)id) {
          XYZ_ENV->cur_robot = XYZ_ENV->robot[i];
#ifdef P3D_PLANNER
          XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;  // Modification Fabien
#endif
          return((void *)(XYZ_ENV->cur_robot));
        }
      PrintError(("MP: p3d_sel_desc_id: wrong id\n"));
      return(NULL);

    case P3D_BODY:
      for (i = 0;i < XYZ_ENV->cur_robot->no;i++)
        if (XYZ_ENV->cur_robot->o[i] == (pp3d_obj)id) {
          XYZ_ENV->cur_robot->ocur = XYZ_ENV->cur_robot->o[i];
          return((void *)(XYZ_ENV->cur_robot->ocur));
        }
      PrintError(("MP: p3d_sel_desc_id: wrong id\n"));
      return(NULL);

    case P3D_TRAJ:
      for (i = 0;i < XYZ_ENV->cur_robot->nt;i++)
        if (XYZ_ENV->cur_robot->t[i] == (pp3d_traj)id) {
          XYZ_ENV->cur_robot->tcur = XYZ_ENV->cur_robot->t[i];
          return((void *)(XYZ_ENV->cur_robot->tcur));
        }
      PrintError(("MP: p3d_sel_desc_id: wrong id\n"));
      return(NULL);

    default:
      PrintError(("MP: p3d_sel_desc_id: wrong type\n"));
      return(NULL);
  }
}

/*
 *  Number of joints current robot
 *
 *  Output : number of joints
 */
int p3d_get_number_joints_current_robot() {
  p3d_rob *robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);

  if (robotPt != NULL) {
    return robotPt->njoints;
  }
  return 0;
}


/*
 *  Number of element of a given type
 *
 *  Input  : type of element
 *
 *  Output : number of elements of that type stored in the current
 *           environment.
 */
int p3d_get_desc_number(int type) {
  switch (type) {
      /* nmuber of environments */
    case P3D_ENV:
      return(XYZ_NUM_ENV);
      /* number of obstacles in XYZ_ENV */
    case P3D_OBSTACLE:
      return(XYZ_ENV ? XYZ_ENV->no : 0);
      /* number of robots in XYZ_ENV */
    case P3D_ROBOT:
      return(XYZ_ENV ? XYZ_ENV->nr : 0);
      /* number of bodies of XYZ_ENV->cur_robot */
    case P3D_BODY:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? XYZ_ENV->cur_robot->no : 0) : 0);
    case P3D_BODIES: {
      int g = 0, i, nr = p3d_get_desc_number(P3D_ROBOT);
      for (i = 0;i < nr;i++) {
        g += (XYZ_ENV ? (XYZ_ENV->robot[i] ? XYZ_ENV->robot[i]->no : 0) : 0);
      }
      return g;
    }    /* number of trajectories in XYZ_ENV->cur_robot */
    case P3D_TRAJ:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? XYZ_ENV->cur_robot->nt : 0) : 0);
    default:
      PrintError(("MP: p3d_sel_desc_id: wrong type\n"));
  }
  return(0);
}

/*************************************************/
/* Fonction recuperant le nom de la description  */
/* courante                                      */
/* In : le type                                  */
/* Out :                                         */
/*************************************************/
char *p3d_get_desc_curname(int type) {
  switch (type) {

    case P3D_ENV:
      return(XYZ_ENV ? XYZ_ENV->name : NULL);
    case P3D_OBSTACLE:
      return(XYZ_ENV ? (XYZ_ENV->ocur ? XYZ_ENV->ocur->name : NULL) : NULL);
    case P3D_ROBOT:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? XYZ_ENV->cur_robot->name : NULL) : NULL);
    case P3D_BODY:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? (XYZ_ENV->cur_robot->ocur ? XYZ_ENV->cur_robot->ocur->name : NULL)
                            : NULL) : NULL);
    case P3D_TRAJ:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ?
                        (XYZ_ENV->cur_robot->tcur ? XYZ_ENV->cur_robot->tcur->name : NULL)
                            : NULL) : NULL);
    default:
      PrintError(("MP: p3d_get_desc_curname: wrong type\n"));
  }
  return(NULL);
}

/*************************************************/
/* Fonction recuperant l'id de la description    */
/* courante                                      */
/* In : le type                                  */
/* Out :                                         */
/*************************************************/
void *p3d_get_desc_curid(int type) {
  switch (type) {
    case P3D_ENV:
      return(XYZ_ENV);
    case P3D_OBSTACLE:
      return(XYZ_ENV ? (void *)XYZ_ENV->ocur : NULL);
    case P3D_ROBOT:
      return(XYZ_ENV ? (void *)XYZ_ENV->cur_robot : NULL);
    case P3D_BODY:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? (void *)XYZ_ENV->cur_robot->ocur : NULL) : NULL);
    case P3D_TRAJ:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? (void *)XYZ_ENV->cur_robot->tcur : NULL) : NULL);
    default:
      PrintError(("MP: p3d_get_desc_curnid: wrong type\n"));
  }
  return(NULL);
}

/***************************************************/
/* Fonction recuperant le numero de la description */
/* courante                                        */
/* In : le type                                    */
/* Out :                                           */
/***************************************************/
int p3d_get_desc_curnum(int type) {
  switch (type) {

    case P3D_ENV:
      return(XYZ_ENV ? XYZ_ENV->num : P3D_NULL_OBJ);
    case P3D_OBSTACLE:
      return(XYZ_ENV ? (XYZ_ENV->ocur ? XYZ_ENV->ocur->num : P3D_NULL_OBJ) : P3D_NULL_OBJ);
    case P3D_ROBOT:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? XYZ_ENV->cur_robot->num : P3D_NULL_OBJ) : P3D_NULL_OBJ);
    case P3D_BODY:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? (XYZ_ENV->cur_robot->ocur ? XYZ_ENV->cur_robot->ocur->num : P3D_NULL_OBJ)
                            : P3D_NULL_OBJ) : P3D_NULL_OBJ);
    case P3D_TRAJ:
      return(XYZ_ENV ? (XYZ_ENV->cur_robot ? (XYZ_ENV->cur_robot->tcur ? XYZ_ENV->cur_robot->tcur->num : P3D_NULL_OBJ) : P3D_NULL_OBJ) : P3D_NULL_OBJ);
    default:
      PrintError(("MP: p3d_get_desc_curnum: wrong type\n"));
  }
  return(0);
}


/************************************************/
/************************************************/
/************************************************/

/***************************************************/
/* Fonction initialisant les variables globales    */
/* d'environnement                                 */
/* In :                                            */
/* Out :                                           */
/***************************************************/
static int p3d_init(void) {
  if (INIT) return(FALSE);
  XYZ_ENV = NULL;
  XYZ_NUM_ENV = 0;
  XYZ_ROBOT = NULL;
  XYZ_OBSTACLES = NULL;

  E_DEF = FALSE;
  O_DEF = FALSE;
  R_DEF = FALSE;
  T_DEF = FALSE;
  INIT  = TRUE;

  p3d_BB_set_mode_close();

  return((INIT = TRUE));
}

/*******************************************/
/* Fonction commencant la description de   */
/* l'environnement courant                 */
/* In : son nom                            */
/* Out :                                   */
/*******************************************/
/* static */
void *p3d_beg_env(char* name) {
  pp3d_env e;

  p3d_init();

  e = MY_ALLOC(p3d_env, 1);
  if (!e) return(NULL);
  memset(e, 0, sizeof(p3d_env));

  e->name    = strdup(name);
  
  e->p3d_file_path    = NULL;
  e->sce_file_path    = NULL;
  
  e->num     = XYZ_NUM_ENV;
  e->box.x1  = P3D_HUGE;
  e->box.y1 = P3D_HUGE;
  e->box.z1 = P3D_HUGE;
  e->box.x2  = -P3D_HUGE;
  e->box.y2 = -P3D_HUGE;
  e->box.z2 = -P3D_HUGE;
  e->no      = e->nof_objs = e->nr = 0;
  e->o       = NULL;
  e->robot    = NULL;
  e->ocur    = NULL;
  e->cur_robot = NULL;

  e->INIT = 1;
  e->modif = 0; /* UI 28052001, init. value */
#ifdef P3D_PLANNER
  if(STAT){
    e->stat = createStat();
  }else{
    e->stat = NULL;
  }
#endif

  e->background_color[0]= 1.0;
  e->background_color[1]= 1.0;
  e->background_color[2]= 1.0;
  e->collisionCloud= NULL;
  e->cloudSize= 0;

  return((void *)(XYZ_ENV = e));
}

/*******************************************/
/* Fonction terminant la description de    */
/* l'environnement courant                 */
/* In :                                    */
/* Out :                                   */
/*******************************************/
static int p3d_end_env(void) {
  int i, rnum;
  configPt q;

  q = p3d_alloc_body_config();

  if (XYZ_ENV->no == 0) {
    PrintWarning(("MP: p3d_end_env: 0 obstacles defined\n"));
  }
  if (XYZ_ENV->nr == 0) {
    PrintWarning(("MP: p3d_end_env: 0 robot defined\n"));
  }

  /* begin modif Carl */
  XYZ_ENV->dmax = sqrt(SQR(XYZ_ENV->box.x1 - XYZ_ENV->box.x2) +
                       SQR(XYZ_ENV->box.y1 - XYZ_ENV->box.y2)) / 100.;
  // modif Pepijn june 2001
  XYZ_ENV->object_tolerance = 0.0; /* default value of the tolerance */
  XYZ_ENV->graphic_dmin = sqrt(SQR(XYZ_ENV->box.x1 - XYZ_ENV->box.x2) +
                               SQR(XYZ_ENV->box.y1 - XYZ_ENV->box.y2)) / 100.; /* initial value */
  /* end modif Carl */

  rnum = p3d_get_desc_curnum(P3D_ROBOT);
  /* initialisation des espaces de travail des robots a celui de l'env */
  /* PrintInfo(("ce sont les boites qui merdent\n")); */
  for (i = 0;i < XYZ_ENV->nr;i++) {
    if (XYZ_ENV->no > 0) {
      XYZ_ENV->robot[i]->box.x1 = XYZ_ENV->box.x1;
      XYZ_ENV->robot[i]->box.x2 = XYZ_ENV->box.x2;
      XYZ_ENV->robot[i]->box.y1 = XYZ_ENV->box.y1;
      XYZ_ENV->robot[i]->box.y2 = XYZ_ENV->box.y2;
      //      XYZ_ENV->cur_robot = XYZ_ENV->robot[i];  Modification Fabien
      p3d_sel_desc_num(P3D_ROBOT, i);

      q[0] = 0.0;
      q[1] = 0.0;
      q[2] = 0.0;
      q[3] = 0.0;
      q[4] = 0.0;
      q[5] = 0.0;

      p3d_set_robot_pos(q);
      p3d_update_robot_pos();
    }
  }
  p3d_sel_desc_num(P3D_ROBOT, rnum);
  p3d_destroy_body_config(q);

  return(TRUE);
}


/**************************************************************/
/* Fonction commencant la description d'un objet p3d rattache */
/* a l'environnement courant, qui devient l'objet courant     */
/* (au moins pour la description...)                          */
/* In : son nom, son type (obstacle, body)                    */
/* Out :                                                      */
/**************************************************************/
/* static */
void *p3d_beg_obj(char *name, int type) {
  pp3d_obj o;

  o = MY_ALLOC(p3d_obj, 1);

  if (!o) return(NULL);

  o->name    = strdup(name);
  o->env     = XYZ_ENV;
  o->type    = type;
  o->jnt     = NULL;
  o->pol     = NULL; // Modification Fabien
  o->np      = 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, o->opos);

  o->box.x1 = P3D_HUGE;
  o->box.x2 = -P3D_HUGE;
  o->box.y1 = P3D_HUGE;
  o->box.y2 = -P3D_HUGE;
  o->box.z1 = P3D_HUGE;
  o->box.z2 = -P3D_HUGE;

  o->BB.xmin = P3D_HUGE;
  o->BB.xmax = -P3D_HUGE;
  o->BB.ymin = P3D_HUGE;
  o->BB.ymax = -P3D_HUGE;
  o->BB.zmin = P3D_HUGE;
  o->BB.zmax = -P3D_HUGE;
  o->concat = 0;
  o->display_mode= P3D_OBJ_DEFAULT_DISPLAY;
#ifdef HRI_PLANNER_GUI
  o->show_pos_area = 0;
  o->trans = 0;
  o->caption_selected = 0;
#endif
  o->contact_surface= 0;

  o->pqpModel= NULL;
  o->pqpPreviousBody= NULL;
  o->pqpID= 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, o->pqpPose);
  o->pqpUnconcatObj= NULL;

#ifdef DPG
  o->nbPointCloud = 0;
  o->pointCloud = NULL;
#endif
  o->robot_part= P3D_NOT_A_PART;
  o->distance_weight= 1.0;
  o->isDeformable= false;
  return((void *)(XYZ_OBSTACLES = o));
}


/********************************************************/
/* Fonction terminant la description de l'objet courant */
/* In :                                                 */
/* Out :                                                */
/********************************************************/
int p3d_end_obj(void) {
  pp3d_obj  *newo = NULL, *oldo = NULL;
  int       num = 0, i, np;
  p3d_matrix4 pos;
//   double dist = 0, x, y, z;
//   int ip, iv, nvert;
//   p3d_poly *p;
  p3d_jnt * jntPt;
  //int newbody = 1;   // modif Juan

  XYZ_OBSTACLES->is_used_in_env_flag = 1;

  /* tsiano a l initialisation des objets on est en mode p3d_mode_close */
  (*p3d_BB_update_BB_obj)(XYZ_OBSTACLES, p3d_mat4IDENTITY);

  if (XYZ_OBSTACLES->type == P3D_OBSTACLE) {
    XYZ_ENV->box.x1 = MIN(XYZ_OBSTACLES->box.x1, XYZ_ENV->box.x1);
    XYZ_ENV->box.x2 = MAX(XYZ_OBSTACLES->box.x2, XYZ_ENV->box.x2);
    XYZ_ENV->box.y1 = MIN(XYZ_OBSTACLES->box.y1, XYZ_ENV->box.y1);
    XYZ_ENV->box.y2 = MAX(XYZ_OBSTACLES->box.y2, XYZ_ENV->box.y2);
    XYZ_ENV->box.z1 = MIN(XYZ_OBSTACLES->box.z1, XYZ_ENV->box.z1);
    XYZ_ENV->box.z2 = MAX(XYZ_OBSTACLES->box.z2, XYZ_ENV->box.z2);
  }
  for (np = 0;np < XYZ_OBSTACLES->np;np++) {
    p3d_get_poly_pos(XYZ_OBSTACLES->pol[np]->poly, pos);
    p3d_mat4Copy(pos, XYZ_OBSTACLES->pol[np]->pos0);
  }



  if (XYZ_OBSTACLES->type == P3D_OBSTACLE) {
    XYZ_OBSTACLES->is_used_in_device_flag = 0;
    num     = XYZ_ENV->no++;
    oldo    = XYZ_ENV->o;
    for (i = 0;i < XYZ_OBSTACLES->np;i++) {
      if (XYZ_OBSTACLES->pol[i]->color == -1)  // <- modif Juan
        XYZ_OBSTACLES->pol[i]->color = Blue;
    }
    XYZ_ENV->ocur = XYZ_OBSTACLES;
  } else if (XYZ_OBSTACLES->type == P3D_BODY) {
    // modif Juan (for definig several bodies without jounts between them)
    for (i = 0;i < XYZ_OBSTACLES->np;i++) {
      if (XYZ_OBSTACLES->pol[i]->color == -1)  // <- modif Juan
        XYZ_OBSTACLES->pol[i]->color = Yellow;
    }
    jntPt  = XYZ_ROBOT->joints[XYZ_ROBOT->njoints];
    XYZ_OBSTACLES->jnt  = jntPt;
    //    if(jntPt->o == NULL) {
    //      newbody = 1;
    XYZ_OBSTACLES->is_used_in_device_flag = 1;
    //    }
    num     = XYZ_ROBOT->no++;
    oldo    = XYZ_ROBOT->o;
    XYZ_ROBOT->ocur = XYZ_OBSTACLES;
    //Concatenation des body
    if(p3d_jnt_set_object(XYZ_ROBOT->joints[XYZ_ROBOT->njoints], XYZ_OBSTACLES)){
      XYZ_OBSTACLES->concat = 1;
    }
    if (XYZ_OBSTACLES->np > 0) {
      double bodyMaxDist = p3d_jnt_compute_max_distance_body_vertex(XYZ_OBSTACLES);
      jntPt->dist = bodyMaxDist > jntPt->dist ? bodyMaxDist : jntPt->dist;
//Modif Mokhtar : Passage a une fonction separee
//       for (ip = 0;ip < XYZ_OBSTACLES->np;ip++) {
//         p = XYZ_OBSTACLES->pol[ip];
// //         if (p->TYPE != P3D_GRAPHIC) {
//           nvert = p3d_get_nb_points(p->poly);
//           for (iv = 1;iv <= nvert;iv++) {
//             /*tsiano p3d_get_vert_poly(p,iv,&x,&y,&z); */
//             p3d_get_point_2_d(p->poly, iv, &x, &y, &z);
//             move_point(p->pos_rel_jnt, &x, &y, &z, 1);
// 
//             dist = sqrt(SQR(x) + SQR(y) + SQR(z));
// 
//             if (dist > jntPt->dist) {
//               jntPt->dist = dist;
//             }
//           }
// //         }
//       }
//end modif
      while (jntPt->prev_jnt != NULL) {
        jntPt = jntPt->prev_jnt;
        if (jntPt->dist < EPS6) {
          jntPt->dist = XYZ_OBSTACLES->jnt->dist;
        } else {
          break;
        }
      }
    }
    /* PrintInfo(("distance finale : %f\n",XYZ_OBSTACLES->jnt->dist)); */
  } else {
    PrintError(("MP: p3d_end_obj: bad object type\n"));
    return(FALSE);
  }

  //  if((XYZ_OBSTACLES->type == P3D_OBSTACLE) || newbody) {
  XYZ_OBSTACLES->o_id = XYZ_ENV->nof_objs++;
  XYZ_OBSTACLES->o_id_in_env = XYZ_OBSTACLES->o_id;
  XYZ_OBSTACLES->num  = num;
  newo    = MY_ALLOC(p3d_obj *, num + 1);

  if (newo) {
    for (i = 0;i < num;i++)  newo[i] = oldo[i];
    newo[num] = XYZ_OBSTACLES;
    MY_FREE(oldo, p3d_obj *, num);
    if (XYZ_OBSTACLES->type == P3D_OBSTACLE) {
      XYZ_ENV->o = newo;
    } else {
      XYZ_ROBOT->o = newo;
    }
  }
  //  }
  // modif Juan (for defining several bodies without joints between them)

// set pointer to poly to obj (modif Juan)
  for (np = 0;np < XYZ_OBSTACLES->np;np++) {
    XYZ_OBSTACLES->pol[np]->p3d_objPt = XYZ_OBSTACLES;
  }
#ifdef DPG
  XYZ_OBSTACLES->nbPointCloud = 0;
  XYZ_OBSTACLES->pointCloud = NULL;
#endif


  return(TRUE);
}

/***************************************************************/
/* Fontcion commencant la description d'un robot p3d rattache */
/* a l'environnement courant, qui devient le robot courant     */
/* (au moins pour la description...)                           */
/* In : son nom                                                */
/* Out :                                                       */
/***************************************************************/
static void *p3d_beg_rob(char* name) {
  pp3d_rob robotPt;
  double dtab[6*3], dtab2[6*2], dtab3[6*2], vel_max[6], accel_max[6], jerk_max[6];
  int i;
  p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;

  robotPt = MY_ALLOC(p3d_rob, 1);
  if (!robotPt)
    return(NULL);

  robotPt->name    = strdup(name);
  robotPt->env     = XYZ_ENV;
  robotPt->ocur    = NULL;
  robotPt->o       = NULL;
  robotPt->joints  = NULL;
  robotPt->j_modif = NULL;
  robotPt->first_joint   = NULL;
  robotPt->first_abs_pos = NULL;
  robotPt->njoints       = -1;
  robotPt->nb_dof        = 0;
  robotPt->nb_user_dof   = 0;
  robotPt->no      = 0;
  robotPt->nt      = 0;
  robotPt->coll    = 0;

  robotPt->nconf   = 0;      /* Modification Fabien */

  robotPt->BB.xmin = P3D_HUGE;
  robotPt->BB.xmax = -P3D_HUGE;
  robotPt->BB.ymin = P3D_HUGE;
  robotPt->BB.ymax = -P3D_HUGE;
  robotPt->BB.zmin = P3D_HUGE;
  robotPt->BB.zmax = -P3D_HUGE;

  robotPt->tcur    = NULL;
  robotPt->t       = NULL;
  robotPt->confcur = NULL;   /* Modification Fabien */
  robotPt->conf    = NULL;   /* Modification Fabien */
  robotPt->cntrt_manager  = NULL;
  robotPt->lpl_type = DEFAULT_LOCAL_PLANNER;
  robotPt->local_method_params = NULL;
  robotPt->GRAPH   = NULL;   /* Modification Fabien */
  robotPt->user_appli = NULL;/* Modification Fabien */
  robotPt->display_mode= P3D_ROB_DEFAULT_DISPLAY;
  robotPt->draw_custom_color=FALSE;
  robotPt->draw_transparent=FALSE;

#ifdef HRI_PLANNER_GUI
  /* Modification Luis */
  robotPt->cam_pos[0]= 0.0;
  robotPt->cam_pos[1]= 0.0;
  robotPt->cam_pos[2]= 0.0;
  robotPt->cam_min_range = 0.0;
  robotPt->cam_max_range = 0.0;
  robotPt->cam_v_angle  = 0.0;
  robotPt->cam_h_angle  = 0.0;
  robotPt->cam_body_index = 0;
  robotPt->angle_range   = 0.0;
  robotPt->max_pos_range = 0.0;
  robotPt->min_pos_range = 0.0;
  robotPt->lookatpoint = NULL;
  robotPt->caption_selected = 0;
  //MY_ALLOC(psp_obs_vertex,1);
  //robotPt->searchBall = MY_ALLOC(p3d_psp_search_element,1);
  //robotPt->searchBall->active=0;
  /* End */
#endif

#ifdef P3D_PLANNER
  if (p3d_GetRefAndMobFrames(robotPt , &RefFramePt, &MobFramePt)) {
    //    p3d_set_weight_for_rotation_distance_metric(robotPt);
    p3d_GetWeightRotaFrameMetric();
  }
#endif


  XYZ_ROBOT  = robotPt;
  R_DEF      = TRUE;

  /* Joint blocked */
  for (i = 0; i < 6; i++) {
    dtab[3*i] = 0.0;                  /* Dof value */
    dtab[3*i+1] = dtab2[2*i] = 0.0;   /* Min and min rand Dof value */
    dtab[3*i+2] = dtab2[2*i+1] = 0.0; /* Max and max rand Dof value */
    dtab3[3*i+1] = dtab3[2*i] = 0.0;
    vel_max[i] = 0.0;
    accel_max[i] = 0.0;
    jerk_max[i] = 0.0;
  }
  p3d_add_desc_jnt_deg(P3D_BASE, p3d_mat4IDENTITY, dtab, P3D_NULL_OBJ,
                       dtab2, 1.0, dtab3, vel_max, accel_max, jerk_max);

  return((void *)(P3D_ROBOT));
}

int return_R_DEF() {
  return R_DEF;
}

/******************************************************/
/* Fonction terminant la description du robot courant */
/* In :                                               */
/* Out :                                              */
/******************************************************/
static int p3d_end_rob(void) {
  pp3d_rob  *newr;
  int       i = 0;

  /* initialisation de la boite englobante du robot */
  p3d_BB_update_BB_rob(XYZ_ROBOT);

  /* allocation des positions de depart et d arrivee du robot */
  XYZ_ROBOT->ROBOT_POS = p3d_alloc_config(XYZ_ROBOT);
  XYZ_ROBOT->ROBOT_GOTO = p3d_alloc_config(XYZ_ROBOT);

  for(i = 0; i < 10; i++){
    XYZ_ROBOT->transitionConfigs[i] = p3d_alloc_config(XYZ_ROBOT);
  }
  XYZ_ROBOT->nTransition = 0;
  XYZ_ROBOT->ikSolPos = NULL; //init of start ikSol
  XYZ_ROBOT->ikSolGoto = NULL; //init of goto ikSol
  XYZ_ROBOT->ikSol = NULL; //init of current ikSol
  for(i = 0; i < 10; i++){
    (XYZ_ROBOT->ikSolTransition)[i] = NULL;
  }
  XYZ_ROBOT->currect_q_inv = p3d_alloc_config(XYZ_ROBOT);  // temporary modif

  /* computation of the array of length coefficients for the distance */
  compute_length_array(XYZ_ROBOT);
  /* on actualise le tableau des robots de l'env courant */
  XYZ_ROBOT->num  = XYZ_ENV->nr++;
  newr    = MY_ALLOC(p3d_rob *, XYZ_ENV->nr);

  if (newr) {
    for (i = 0;i < XYZ_ENV->nr - 1;i++)
      newr[i] = XYZ_ENV->robot[i];
    newr[XYZ_ENV->nr-1] = XYZ_ROBOT;
    MY_FREE(XYZ_ENV->robot, p3d_rob *, XYZ_ENV->nr - 1);
    XYZ_ENV->robot = newr;
  }

  // Set the active robot
  // this can be overriden in the UI or in the p3d file
  XYZ_ENV->active_robot = XYZ_ROBOT;

  // Set the current robot
  XYZ_ENV->cur_robot = XYZ_ROBOT;

#ifdef P3D_PLANNER
  XYZ_GRAPH = XYZ_ENV->cur_robot->GRAPH;  // Modification Fabien
#endif

  /* initialization fo the prejacobian matrix (EF) */
  p3d_jacInitialization(XYZ_ROBOT);
	
#ifdef P3D_CONSTRAINTS
  /* constraints */
  XYZ_ROBOT->cntrt_manager = p3d_create_cntrt_manager(XYZ_ROBOT->nb_dof);
#endif
#ifdef MULTIGRAPH
  XYZ_ROBOT->mg = MY_ALLOC(p3d_multiGraph,1);
  p3d_initMultiGraph(XYZ_ROBOT, XYZ_ROBOT->mg);
#endif

#ifdef MULTILOCALPATH
  /*initialisation des variables pour les multiLocalPath*/
  XYZ_ROBOT->mlp = MY_ALLOC(p3d_multiLocalPath,1);
  XYZ_ROBOT->mlp->nblpGp = 0;
  XYZ_ROBOT->mlp->mlpJoints = NULL;
  XYZ_ROBOT->mlp->active = NULL;
  XYZ_ROBOT->mlp->t = NULL;
#endif

#ifdef LIGHT_PLANNER
  XYZ_ROBOT->graspNbJoints = 0;
  XYZ_ROBOT->graspJoints = NULL;
  XYZ_ROBOT->baseJnt = NULL;
  XYZ_ROBOT->curObjectJnt = NULL;
  XYZ_ROBOT->relativeZRotationBaseObject = 0.0;
  XYZ_ROBOT->isUserDof = MY_ALLOC(int, XYZ_ROBOT->nb_dof);
  for(int k = 0, i = 0; i < XYZ_ROBOT->njoints + 1; i++){
    p3d_jnt * jntPt = XYZ_ROBOT->joints[i];
    for(int j = 0; j < jntPt->dof_equiv_nbr; j++, k++) {
      XYZ_ROBOT->isUserDof[k] = p3d_jnt_get_dof_is_user(jntPt, j);
    }
  }
  XYZ_ROBOT->nbCcCntrts = 0;
  XYZ_ROBOT->ccCntrts = NULL;
  XYZ_ROBOT->openChainConf = p3d_alloc_config(XYZ_ROBOT);
  XYZ_ROBOT->closedChainConf = p3d_alloc_config(XYZ_ROBOT);
  for(int k = 0; k < 4; k++){
    XYZ_ROBOT->preComputedGraphs[k] = 0;
  }
 #if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
  XYZ_ROBOT->nbFkCntrts = 0;
  XYZ_ROBOT->fkCntrts = NULL;
 #endif
#endif

#if defined(LIGHT_PLANNER)
  XYZ_ROBOT->isCarryingObject= FALSE;
//  XYZ_ROBOT->carriedObject= NULL;
  XYZ_ROBOT->configCostThreshold = 0.0;
  XYZ_ROBOT->inhibitCollisionTolerance = FALSE;
#endif

#ifdef LIGHT_PLANNER
 XYZ_ROBOT->armManipulationData = new std::vector<ArmManipulationData>;
#endif

#ifdef DPG
  XYZ_ROBOT->nbDpgCells = 0;
  XYZ_ROBOT->dpgCells = NULL;
#endif
 p3d_update_robot_pos();

 return(TRUE);
}


/****************/
/* Trajectoires */
/****************/

/********************************************************/
/* Fonction commencant la description d'une trajectoire */
/* In : le nom de la trajectoire                        */
/* Out :                                                */
/********************************************************/
static void *p3d_beg_traj(char* name) {
  pp3d_traj t;

  t = MY_ALLOC(p3d_traj, 1);
  if (!t) return(NULL);

  t->name       = strdup(name);
  t->file       = NULL;  // Modification Fabien
  t->num        = XYZ_ENV->cur_robot->nt;
  t->rob        = XYZ_ENV->cur_robot;
  t->nlp        = 0;
  t->courbePt   = NULL;
#ifdef DPG
  t->isOptimized = false;
  t->savelpNum = 0;
  t->trajInGraph = NULL;
#endif
  return((void *)(XYZ_TRAJS = t));
}


/********************************************************/
/* Fonction terminant la description d'une trajectoire  */
/* In : le nom de la trajectoire                        */
/* Out :                                                */
/********************************************************/
static int p3d_end_traj(void) {
  p3d_rob   *robotPt = XYZ_TRAJS->rob;
  pp3d_traj  *newt;
  pp3d_traj  *newglobt;
  int       i;

  /* on actualise le tableau des traj du robot */
  robotPt->nt++;
  newt    = MY_ALLOC(p3d_traj *, robotPt->nt);
  if (XYZ_TRAJS != NULL) {
#ifdef P3D_PLANNER
    XYZ_TRAJS->range_param = p3d_compute_traj_rangeparam(XYZ_TRAJS);
#endif
  }

  if (newt) {
    for (i = 0;i < robotPt->nt - 1;i++)  newt[i] = robotPt->t[i];
    newt[robotPt->nt-1] = XYZ_TRAJS;
    MY_FREE(robotPt->t, p3d_traj *, robotPt->nt - 1);
    robotPt->t = newt;
  }

  robotPt->tcur = XYZ_TRAJS;

  /* begin modif. Carl 28032001 */
  /* we add traj also the the global array containing all trajectories */
  nof_trajs++;
  newglobt    = MY_ALLOC(pp3d_traj, nof_trajs);
  if (newglobt) {
    for (i = 0;i < nof_trajs - 1;i++)  newglobt[i] = XYZ_TRAJ[i];
    newglobt[nof_trajs-1] = XYZ_TRAJS;
    XYZ_TRAJS->id = nof_trajs - 1;
    MY_FREE(XYZ_TRAJ, pp3d_traj, nof_trajs - 1);
    XYZ_TRAJ = newglobt;
  }
  /*  end  modif. Carl 28032001 */

  return(TRUE);
}

/************************************************/


/* *************************************************** *
 * int trj_set_null(int traj_id)                       *
 * Function setting entry to NULL pointer              *
 * ARGS IN: traj_id     unique identifier of           *
 *                      the trajectory to delete       *
 * RETURNS: TRUE if identifier exists, FALSE otherwise *
 * REMARKS: added Carl 28032001                        *
 * *************************************************** */
int trj_set_null(int traj_id) {
  int i, all_null = TRUE;
  int trj_success = FALSE;

  /* set pointer NULL, but keep the entry */
  if (traj_id < nof_trajs) {
    XYZ_TRAJ[traj_id] = NULL;
    trj_success = TRUE;
  }

  /* clean up all trajectory pointers when none are left */
  for (i = 0;(i < nof_trajs) && (all_null);i++) {
    all_null = all_null && (XYZ_TRAJ[i] == NULL);
  }
  if (all_null) {
    MY_FREE(XYZ_TRAJ, pp3d_traj, nof_trajs);
    XYZ_TRAJ = NULL;
    nof_trajs = 0;
  }

  return trj_success;
}

/*****************************************************/
/* Fonction calculant les nouvelles coordonnees d'un */
/* point avec sa matrice de passage                  */
/*****************************************************/
/* static */
void move_point(p3d_matrix4 pos, double *x, double *y, double *z, int point) {
  double x1 = *x, y1 = *y, z1 = *z;

  *x = pos[0][0] * x1 + pos[0][1] * y1 + pos[0][2] * z1 + pos[0][3] * ((double) point);
  *y = pos[1][0] * x1 + pos[1][1] * y1 + pos[1][2] * z1 + pos[1][3] * ((double) point);
  *z = pos[2][0] * x1 + pos[2][1] * y1 + pos[2][2] * z1 + pos[2][3] * ((double) point);
}

#ifdef MULTIGRAPH

/** \brief Alloc and init a multiGraph.
 \param robot the current robot
 \param mg the multigraph to init
 */
void p3d_initMultiGraph(p3d_rob* robot, p3d_multiGraph* mg){
  mg->envName = XYZ_ENV->name;
  mg->robotName = robot->name;
  mg->nbGraphs = 0;
  mg->graphs = NULL;
  mg->active = NULL;
  mg->mgJoints = NULL;
  mg->usedJoint = MY_ALLOC(int, robot->njoints + 1);
  for(int i = 0; i < robot->njoints + 1; i++){
    mg->usedJoint[i] = 0;
  }
  mg->involvesCp = 1;
  mg->fsg = NULL;
  return;
}

/** \brief Clone a multiGraph.
 \param robot the current robot
 \param src the MultiGraph to clone
 \return The cloned multiGraph
 */
p3d_multiGraph* p3d_cloneMultiGraph(p3d_rob* robot, p3d_multiGraph* src){
  p3d_multiGraph* mg = NULL;
  /*initialisation des variables pour les multigraphs*/
  mg = MY_ALLOC(p3d_multiGraph,1);
  mg->envName = src->envName;
  mg->robotName = src->robotName;
  mg->nbGraphs = src->nbGraphs;
  mg->graphs = MY_ALLOC(p3d_graph *, src->nbGraphs);
  for(int i = 0; i < src->nbGraphs; i++){
    mg->graphs[i] = src->graphs[i];
  }
  mg->active = MY_ALLOC(int, src->nbGraphs);
  for(int i = 0; i < src->nbGraphs; i++){
    mg->active[i] = src->active[i];
  }
  mg->mgJoints = MY_ALLOC(p3d_multiGraphJoint *, src->nbGraphs);
  for(int i = 0; i < src->nbGraphs; i++){
    mg->mgJoints[i] = p3d_cloneMultiGraphJoint(src->mgJoints[i]);
  }
  mg->usedJoint = MY_ALLOC(int, robot->njoints + 1);
  for(int i = 0; i < robot->njoints + 1; i++){
    mg->usedJoint[i] = src->usedJoint[i];
  }
  mg->involvesCp = src->involvesCp;
  mg->fsg = src->fsg;
  return mg;
}

static p3d_multiGraphJoint * p3d_cloneMultiGraphJoint(p3d_multiGraphJoint * src){
  p3d_multiGraphJoint * dst = MY_ALLOC(p3d_multiGraphJoint, 1);
  dst->nbJoints = src->nbJoints;
  dst->joints = MY_ALLOC(int, src->nbJoints);
  dst->cntrts = MY_ALLOC(int, src->nbJoints);
  for(int i = 0; i < src->nbJoints; i++){
    dst->joints[i] = src->joints[i];
    dst->cntrts[i] = src->cntrts[i];
  }
  return dst;
}

/** \brief add a multi Graph joint in the robot structure.
    \param r the current robot
    \param nbJoints the number of joints
    \param joints the joint nums
    \return TRUE if the operation succeed FALSE otherwise
*/
int p3d_set_multi_graph_data(p3d_rob* r, int nbJoints, int *joints){
  if (nbJoints != 0){
    r->mg->nbGraphs++;
    if(r->mg->nbGraphs==1){//si c'est le premier
      r->mg->graphs = MY_ALLOC(p3d_graph*, 10);
      r->mg->active = MY_ALLOC(int, 10);
      r->mg->mgJoints = MY_ALLOC(p3d_multiGraphJoint*, 10);
      for(int i = 0; i < 10; i++){//initialisation
        r->mg->graphs[i] = NULL;
        r->mg->active[i] = -1;
        r->mg->mgJoints[i] = NULL;
      }
    }
    r->mg->active[r->mg->nbGraphs-1] = 1;
    r->mg->mgJoints[r->mg->nbGraphs-1] = MY_ALLOC(p3d_multiGraphJoint, 1);
    (r->mg->mgJoints[r->mg->nbGraphs-1])->nbJoints = nbJoints;
    (r->mg->mgJoints[r->mg->nbGraphs-1])->joints = MY_ALLOC(int, nbJoints);
    (r->mg->mgJoints[r->mg->nbGraphs-1])->cntrts = MY_ALLOC(int, nbJoints);
    for(int i = 0; i < nbJoints; i++){
      (r->mg->mgJoints[r->mg->nbGraphs-1])->joints[i] = joints[i];
      if(r->mg->usedJoint[joints[i]] == 0){
        r->mg->usedJoint[joints[i]] = 1;
        p3d_cntrt * existingCntrt = getJntFixedCntrt(r->cntrt_manager, joints[i]);
        if (existingCntrt == NULL){
          if((r->joints[joints[i]])->type != P3D_BASE && (r->joints[joints[i]])->type != P3D_FIXED){//si ce n'est pas le joint base ni un joint fixe
            //on cree une contrainte pour chaque joint et on la desactive
            int Jpasiv[1] = {joints[i]};
            double Dval[1] = {(r->joints[joints[i]])->dof_data[0].v};
            if (p3d_constraint("p3d_fixed_jnt", -1, Jpasiv, -1, NULL,-1, Dval, -1, NULL, -1, 0)){
              (r->mg->mgJoints[r->mg->nbGraphs - 1])->cntrts[i] = r->cntrt_manager->cntrts[r->cntrt_manager->ncntrts - 1]->num;
            }
          }else{
            (r->mg->mgJoints[r->mg->nbGraphs - 1])->cntrts[i] = -1;
          }
        }else{
          (r->mg->mgJoints[r->mg->nbGraphs - 1])->cntrts[i] = -1;
        }
      }else{// si le joint a deja ete declare
        return FALSE;
      }
    }
  }else{
    return FALSE;
  }
  return TRUE;
}
#endif

#ifdef LIGHT_PLANNER
/** \brief add a group of joints in the robot structure to switch between the bounding box and the real geometry.
    \param r the current robot
    \param nbJoints the number of joints
    \param joints the joint nums
    \return TRUE if the operation succeed FALSE otherwise
*/
int p3d_set_removable_bb_for_grasp(p3d_rob* r, int nbJoints, int *joints){
  if (nbJoints != 0){
    if (r->graspJoints == NULL){
      r->graspJoints = MY_ALLOC(p3d_jnt*, nbJoints);
      r->graspNbJoints = nbJoints;
      for(int i = 0; i < nbJoints; i++){
        r->graspJoints[i] = r->joints[joints[i]];
      }
    }else{
      return FALSE;
    }
  }else{
    return FALSE;
  }
  return TRUE;
}

/** \brief set the arm data for the manipulation planner class
    \param r the current robot
    \param data the data: ccntrtId, mlpGroupId, handType, virtualObjJntId
    \return TRUE if the operation succeed FALSE otherwise
*/
int p3d_set_arm_data(p3d_rob* r, int *data){
  ArmManipulationData armData;
  armData.setId(r->armManipulationData->size());
  armData.setCcCntrt(r, data[0]);
#ifdef FK_CNTRT
  armData.setFkCntrt(p3d_create_FK_cntrts(r, armData.getCcCntrt()));
#endif
#ifdef GRASP_PLANNING
  armData.setHandProperties(data[1]);
#endif
  armData.setManipulationJnt(r, data[2]);
  r->armManipulationData->push_back(armData);
  return TRUE;
}
#endif



#ifdef MULTILOCALPATH
/** \brief add a multi localpath joint in the robot structure.
    \param r the current robot
    \param nbJoints the number of joints
    \param joints the joint nums
    \return TRUE if the operation succeed FALSE otherwise
 */
int p3d_set_multi_localpath_group(p3d_rob* r, int nbJoints, int *joints, int activated){
	int nbDofs = 0;

  if (nbJoints != 0){
    r->mlp->nblpGp++;
    if (r->mlp->nblpGp > MAX_MULTILOCALPATH_NB) {
      printf("p3d_env.c, localpath.h : p3d file has too much multigraphs, please change MAX_MULTIGRAPH_NB and re-build\n");
      return FALSE;
    }
    if(r->mlp->nblpGp==1){//si c'est le premier
      r->mlp->mlpJoints = MY_ALLOC(p3d_multiLocalPathJoint*, MAX_MULTILOCALPATH_NB);
      r->mlp->t = MY_ALLOC(p3d_traj*, MAX_MULTILOCALPATH_NB);
      r->mlp->active = MY_ALLOC(int, MAX_MULTILOCALPATH_NB);

    }
    r->mlp->active[r->mlp->nblpGp-1] = activated;
    r->mlp->t[r->mlp->nblpGp-1] = NULL;
    (r->mlp->mlpJoints[r->mlp->nblpGp-1]) = MY_ALLOC(p3d_multiLocalPathJoint, 1);
    (r->mlp->mlpJoints[r->mlp->nblpGp-1])->nbJoints = nbJoints;
    (r->mlp->mlpJoints[r->mlp->nblpGp-1])->joints = MY_ALLOC(int, nbJoints);
    //(r->mg->mgJoints[r->mg->nbGraphs-1])->gpName --> not need to init
    (r->mlp->mlpJoints[r->mlp->nblpGp-1])->lplType = (p3d_localpath_type)-1;
    (r->mlp->mlpJoints[r->mlp->nblpGp-1])->gpType = (p3d_group_type)-1;
    (r->mlp->mlpJoints[r->mlp->nblpGp-1])->local_method_params =  NULL;

    for(int i = 0; i < nbJoints; i++){
      (r->mlp->mlpJoints[r->mlp->nblpGp-1])->joints[i] = joints[i];
			nbDofs += r->joints[r->mlp->mlpJoints[r->mlp->nblpGp-1]->joints[i]]->user_dof_equiv_nbr;
    }
   (r->mlp->mlpJoints[r->mlp->nblpGp-1])->nbDofs = nbDofs;
   p3d_multiLocalPath_set_groupToPlan(r, r->mlp->nblpGp-1, 0, FALSE);
  }else{
    return FALSE;
  }
  return TRUE;
}

//extern p3d_group_type p3d_group_getid_group(const char * name);
//extern p3d_group_type p3d_group_getid_group(const char *);
/** \brief add data to multi Graph joint in the robot structure.
    \param r the current robot
    \param name the name of the sub-robot (used for to find the model)
    \param lp the localpath used
    \param dtab parameters of softMotion localpath
    \return TRUE if the operation succeed FALSE otherwise
 */
int p3d_set_multi_localpath_data(p3d_rob* r, const char* gp_name_in, const char* gp_type_in, char* lpl_type_in, double *dtab)
{
  psoftMotion_str softMotion_params = NULL;
  p3d_localpath_type lpl_type = (p3d_localpath_type)-1;
  p3d_group_type gp_type = (p3d_group_type)P3D_NULL_OBJ;
  int nblpGp = r->mlp->nblpGp;

  lpl_type = p3d_local_getid_planner(lpl_type_in);
  gp_type = p3d_group_getid_group(gp_type_in);

  if(gp_type  == P3D_NULL_OBJ) {
    printf("p3d_env.c : %s group is set to NULL\n", gp_type_in);
  }

  (r->mlp->mlpJoints[nblpGp-1])->gpType = gp_type;
  (r->mlp->mlpJoints[nblpGp-1])->lplType = lpl_type;
  strcpy(r->mlp->mlpJoints[nblpGp-1]->gpName, gp_name_in);

  if(strcmp(lpl_type_in, "Soft-Motion")==0) {

    softMotion_params = lm_get_softMotion_lm_param_multilocalpath(r, nblpGp-1);
    if (softMotion_params != NULL){
      PrintWarning(("softMotion params already initialized\n"));
      return FALSE;
    }
    softMotion_params = lm_create_softMotion(r, nblpGp-1);
    if (softMotion_params != NULL){
      r->mlp->mlpJoints[nblpGp-1]->local_method_params =
          lm_append_to_list(r->mlp->mlpJoints[nblpGp-1]->local_method_params, (void*)softMotion_params, SOFT_MOTION);
    }
//    printf("%s steering method for %s is called \n",lpl_type_in, gp_type_in);
    printf("%s steering method for %s is called \n",lpl_type_in, gp_name_in);
  } else if (strcmp(lpl_type_in, "R&S+linear")==0) {
    printf("%s steering method for %s is called \n",lpl_type_in, gp_name_in);
  } else if (strcmp(lpl_type_in, "Linear")==0) {
    printf("%s steering method for %s is called \n",lpl_type_in, gp_name_in);


  } else {
    printf("Localpath %s unknowed for multigraph\n", lpl_type_in);
    return FALSE;
  }

  return TRUE;
}
#endif

#ifdef DPG
void p3d_compute_static_objects_point_cloud(p3d_env* env, double step){
  for(int i = 0; i < env->no; i++){
    p3d_compute_object_point_cloud(env->o[i], step);
  }
}
void p3d_compute_all_robots_bodies_point_cloud(p3d_env* env, double step){
  for(int i = 0; i < env->nr; i++){
    p3d_compute_robot_bodies_point_cloud(env->robot[i], step);
  }
}
void p3d_compute_robot_bodies_point_cloud(p3d_rob* robot, double step){
  for(int i = 0; i <= robot->njoints; i++){
    if(robot->joints[i]->o){
      p3d_compute_object_point_cloud(robot->joints[i]->o, step);
    }
  }
}
void p3d_compute_object_point_cloud(p3d_obj* obj, double step){
  for(int i = 0; i < obj->np; i++){
    if(obj->pol[i]->TYPE != P3D_GRAPHIC){
      p3d_polyhedre* poly = obj->pol[i]->poly;
      p3d_vector3 the_points[poly->nb_points];
      for(unsigned int j = 0; j < poly->nb_points; j++){
        the_points[j][0] = poly->the_points[j][0];
        the_points[j][1] = poly->the_points[j][1];
        the_points[j][2] = poly->the_points[j][2];
        if (obj->type == P3D_OBSTACLE){//real point position
          p3d_xformPoint(obj->pol[i]->pos0, poly->the_points[j], the_points[j]);
        }else{
          p3d_matrix4 inv_pos, mat;
          p3d_matInvertXform( obj->jnt->pos0, inv_pos );
          p3d_matMultXform(inv_pos, obj->pol[i]->pos0, mat);
          p3d_xformPoint(mat, poly->the_points[j], the_points[j]);
        }
      }
      for(unsigned int j = 0; j < poly->nb_faces; j++){
        unsigned int nbPoints = 0;
        p3d_vector3* tmp = sample_triangle_surface(the_points[poly->the_faces[j].the_indexs_points[0] - 1], the_points[poly->the_faces[j].the_indexs_points[1] - 1], the_points[poly->the_faces[j].the_indexs_points[2] -1 ], step, &nbPoints);
        obj->pointCloud = MY_REALLOC(obj->pointCloud, p3d_vector3, obj->nbPointCloud, obj->nbPointCloud + nbPoints);
        for(unsigned int k = 0; k < nbPoints; k++){
          obj->pointCloud[obj->nbPointCloud + k][0] = tmp[k][0];
          obj->pointCloud[obj->nbPointCloud + k][1] = tmp[k][1];
          obj->pointCloud[obj->nbPointCloud + k][2] = tmp[k][2];
        }
        obj->nbPointCloud += nbPoints;
        free(tmp);
      }
    }
  }
}
#endif

//! This functions prints some info a p3d_obj (for debug purpose).
//! \return 0 in case of success, 1 otherwise
int p3d_print_obj_info(p3d_obj *o)
{
  if(o==NULL)  {
    printf("%s: %d: p3d_print_obj_info(): input p3d_obj* is NULL\n",__FILE__,__LINE__);
    return 1;
  }

  printf("  obj: %s\n", o->name);
  printf("  [\n");
  printf("\t num= %d\n", o->num);
  printf("\t o_id= %d\n", o->o_id);
  printf("\t o_id_in_env= %d\n", o->o_id_in_env);
  printf("\t geo_id= %d\n", o->geo_id);
  printf("\t GRAPHIC_TYPE: ");
  switch(o->GRAPHIC_TYPE)
  {
    case P3D_DEACTIVATED_OBSTACLE:   printf("P3D_DEACTIVATED_OBSTACLE\n");   break;
    case P3D_ACTIVATED_OBSTACLE:     printf("P3D_ACTIVATED_OBSTACLE\n");     break;
    case P3D_ADDABLE_OBSTACLE:       printf("P3D_ADDABLE_OBSTACLE\n");       break;
    case P3D_ADDED_OBSTACLE:         printf("P3D_ADDED_OBSTACLE\n");         break;
    case P3D_REAL_OBJECT:            printf("P3D_REAL_OBJECT\n");            break;
    case P3D_GRAPHIC_OBJECT:         printf("P3D_GRAPHIC_OBJECT\n");         break;
    case P3D_GHOST_OBJECT:           printf("P3D_GHOST_OBJECT\n");           break;
    default:                         printf("undefined\n");                  break;
  }

  printf("\t robot_part: ");
  switch(o->robot_part)
  {
    case P3D_NOT_A_PART:      printf("P3D_NOT_A_PART\n");     break;
    case P3D_HAND_PART:       printf("P3D_HAND_PART\n");      break;
    case P3D_ARM_PART:        printf("P3D_ARM_PART\n");       break;
    case P3D_BASE_PART:       printf("P3D_BASE_PART\n");      break;
    case P3D_FINGER_PART:     printf("P3D_FINGER_PART\n");    break;
    case P3D_FINGERTIP_PART:  printf("P3D_FINGERTIP_PART\n"); break;
    default:                  printf("undefined\n");          break;
  }
  printf("\t distance_weight= %f\n", o->distance_weight);

  printf("\t is_used_in_device_flag= %d\n", o->is_used_in_device_flag);
  printf("\t is_used_in_env_flag= %d\n", o->is_used_in_env_flag);
  printf("\t type= %d \n",o->type);
  printf("\t np= %d \n",o->np);
  printf("\t concat= %d\n", o->concat);
  printf("\t contact_surface= %d\n", o->contact_surface);
  printf("  ]\n");
 
  return 0;
}

//! This functions prints some info about environment obstacles and robot bodies (for debug purpose).
//! \return 0 in case of success, 1 otherwise
int p3d_print_env_info()
{
  int i, j;

  printf("OBSTACLES: \n");
  for(i=0; i<XYZ_ENV->no; ++i)  {
    p3d_print_obj_info(XYZ_ENV->o[i]);
  }

  printf("ROBOTS: \n");
  for(i=0; i<XYZ_ENV->nr; ++i) {
    printf("robot: %s\n",XYZ_ENV->robot[i]->name);
    printf(" {\n");
    for(j=0; j<XYZ_ENV->robot[i]->no; ++j) {
       p3d_print_obj_info(XYZ_ENV->robot[i]->o[j]);
    }
    printf(" }\n");
  }

  return 0;
}


//! Builds a plane (p3d_plane) from the coordinates of three points.
//! The equation parameters are defined as:
//! normale.p + d = 0 for every point belonging to the point (with "." the dot product).
p3d_plane p3d_plane_from_points(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
  p3d_plane plane;
  p3d_vector3 p1p2, p1p3, normal;
  p3d_vectSub(p2, p1, p1p2);
  p3d_vectSub(p3, p1, p1p3);
  p3d_vectXprod(p1p2, p1p3, normal);
  p3d_vectNormalize(normal, plane.normale);
  
  plane.d= -plane.normale[0]*p1[0] - plane.normale[1]*p1[1] - plane.normale[2]*p1[2];
  
  return plane;
}
 

//! Returns 1 if the point is above the plane (i.e. the side where is pointing the normal)
//! 0 otherwise
int p3d_is_point_above_plane(p3d_vector3 point, p3d_plane plane)
{
  if( p3d_vectDotProd(plane.normale, point) + plane.d > 0 )
    return 1;
  else
    return 0;
}

//! Sets the current collision cloud of the environment from an array of p3d_vector3.
//! \return 0 in case of success, 1 otherwise
 int p3d_set_collision_cloud(p3d_vector3* points, int nbPoints)
{
  if(points==NULL)
  {
    printf("%s: %d: p3d_set_collision_cloud(): input p3d_vector3* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  int i; 

  if(XYZ_ENV->collisionCloud!=NULL)
  {
    free(XYZ_ENV->collisionCloud);
  }
 
  XYZ_ENV->collisionCloud= (p3d_vector3*) malloc(sizeof(p3d_vector3)*nbPoints);
  XYZ_ENV->cloudSize= nbPoints;

  for(i=0; i<XYZ_ENV->cloudSize; ++i)
  {
    p3d_vectCopy(points[i], XYZ_ENV->collisionCloud[i]);
  }

  return 0;
}

//! Prepares a body to make it deformable i.e. a body that will be rescaled according to the DOF
//! of a prismatic joint it is related to.
//! \param name name of the body (as it will appear in the .macro file of the robot)
//! \return 0 in case of success, 1 otherwise
int p3d_make_body_deformable(char *name) {
  int i;
  unsigned int j;
  p3d_obj *obj= NULL;

  obj = p3d_get_body_by_name(name);
  if (obj == NULL) {
    printf("%s: %d: p3d_adjust_deformable_body(): no body with name \"%s\" is declared.\n", __FILE__, __LINE__, name);
    return 1;
  } 

  obj->isDeformable= true;

  // make a copy of the original vertex positions:
  for(i=0; i<obj->np; ++i) {
    obj->pol[i]->poly->originalPoints= (p3d_vector3*) malloc(obj->pol[i]->poly->nb_points*sizeof(p3d_vector3));
    for(j=0; j<obj->pol[i]->poly->nb_points; ++j) {
      p3d_vectCopy(obj->pol[i]->poly->the_points[j], obj->pol[i]->poly->originalPoints[j]);
    }
  }

  return 0;
}


//! Scales a body to make it fit the length of its associated "bone" (the segments between its joint and the next joint).
//! \param name name of the body (as it will appear in the .macro file of the robot)
//! \return 0 in case of success, 1 otherwise
int p3d_adjust_deformable_body(p3d_obj *obj, int opengl_context) {
  if(obj==NULL) {
    printf("%s: %d: p3d_adjust_deformable_body(): input p3d_obj is NULL.\n", __FILE__, __LINE__);
    return 1;
  }
  if(obj->isDeformable==false) {
    return 1;
  }

  int i;
  unsigned int j;
  double d;
  double lengthShift, length, scale;
  p3d_vector3 p1, p2, direction, t;
  p3d_jnt *joint= NULL, *translationJoint= NULL, *prevJoint= NULL, *nextJoint= NULL;

  joint= obj->jnt;

  if(joint==NULL) {
    printf("%s: %d: p3d_adjust_deformable_body(): body named \"%s\" should have an associated joint.\n", __FILE__, __LINE__, obj->name);
    return 1;
  }

  translationJoint=  joint->next_jnt[0];
  prevJoint= joint->prev_jnt;
  nextJoint= joint->next_jnt[0]->next_jnt[0];

  if(translationJoint->type!=P3D_TRANSLATE) {
    printf("%s: %d: p3d_adjust_deformable_body(): joint named \"%s\" should be of type P3D_TRANSLATE.\n", __FILE__, __LINE__, joint->name);
    return 1;
  }

  p3d_mat4ExtractTrans(prevJoint->abs_pos, p1);
  p3d_mat4ExtractTrans(nextJoint->abs_pos, p2);
  p3d_vectSub(p2, p1, t);
  length= p3d_vectNorm(t);

  lengthShift= translationJoint->dof_data[0].v;

  direction[0]= 0;
  direction[1]= 1;
  direction[2]= 0;

  scale= length/(length-lengthShift);

  for(i=0; i<obj->np; ++i) {

    for(j=0; j<obj->pol[i]->poly->nb_points; ++j) {
      d= p3d_vectDotProd(obj->pol[i]->poly->originalPoints[j], direction);

      obj->pol[i]->poly->the_points[j][0]= obj->pol[i]->poly->originalPoints[j][0] + d*(scale-1.0)*direction[0];
      obj->pol[i]->poly->the_points[j][1]= obj->pol[i]->poly->originalPoints[j][1] + d*(scale-1.0)*direction[1];
      obj->pol[i]->poly->the_points[j][2]= obj->pol[i]->poly->originalPoints[j][2] + d*(scale-1.0)*direction[2];
    }
    g3d_delete_poly(obj->pol[i],0,opengl_context);
    g3d_delete_poly(obj->pol[i],1,opengl_context);
    g3d_delete_poly(obj->pol[i],2,opengl_context);
    g3d_init_poly(obj->pol[i],0,opengl_context);
    g3d_init_poly(obj->pol[i],1,opengl_context);
    g3d_init_poly(obj->pol[i],2,opengl_context);
  }

  return 0;
}

