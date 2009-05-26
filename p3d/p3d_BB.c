/****************************************************************************/
/*!
 *  \file p3d_BB.c
 *
 *    \brief Bounding box management.
 *
 *    These functions defined the computation method of bounding box,
 *    and the selection of the objets to take into account for this
 *    computation.
 */
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"


/***************************************************************************
 ***************************************************************************
 * Variables for the management of the current selectioon of bounding box
 */

/*! \brief Structure to store the data of each environment
 *  \internal
 */
typedef struct BB_env {
  /*! \brief Number of the environment */
  int p3d_env_id;

  /*! \brief Store the default selection of bounding box
   *  (::default_BB_handlePt)
   */
  p3d_BB_handle * default_BB_handlePt;

  /*! \brief Store the current selectionof bounding box.
   *  (::cur_BB_handlePt)
   */
  p3d_BB_handle * cur_BB_handlePt;
} p3d_BB_env;

/*! \brief Array to store the data for each environment.
 *  \internal
 */
static p3d_BB_env * BB_envs = NULL;

/*! \brief Number of environment stored in ::BB_envs.
 *  \internal
 */
static int nof_BB_envs = 0;

/*! \brief Index of the current environment used in ::BB_envs.
 *  \internal
 */
static int index_cur_BB_envs = -1;

/*! \brief Current selection of bounding box.
 *  \internal
 */
static p3d_BB_handle * cur_BB_handlePt = NULL;

/*! \brief Selection of collision pairs that can be used when none
 *         current selection is defined
 *  \internal
 */
static p3d_BB_handle * default_BB_handlePt = NULL;

/*! \brief Selection of the type of bounding box selection.
 *  \internal
 */
static p3d_BB_selection_type type_BB = ACTIVATE_BB_ALL;

/*! \brief Selection of the type of bounding box computation.
 *  \internal
 */
static p3d_BB_compute_type type_compute_BB = COMPUTE_BB_CLOSE;


/* static functions */
static void init_BB0_obj(p3d_obj *obj);
static void p3d_BB_get_BB_poly_null(p3d_poly *p, double *x1, double *x2,
                                    double *y1, double *y2,
                                    double *z1, double *z2);
static void p3d_BB_update_BB_obj_null(p3d_obj *obj, p3d_matrix4 mat);


/* pointer to function to choose the type of bounding box computation */
p3d_BB_update_BB_fct_type p3d_BB_update_BB_obj;
p3d_BB_get_BB_poly_fct_type p3d_BB_get_BB_poly;


/*
 *  Choose the type of bounding box selection
 *
 *  Input:  the method type
 */
void p3d_BB_set_selection_method(p3d_BB_selection_type type) {
  if (type_BB == DEACTIVATE_BB) {
    p3d_BB_set_computation_method(type_compute_BB);
  }
  if ((type >= ACTIVATE_BB_ALL) && (type <= DEACTIVATE_BB)) {
    type_BB = type;
  }
  if (type_BB == DEACTIVATE_BB) {
    p3d_BB_update_BB_obj = p3d_BB_update_BB_obj_null;
    p3d_BB_get_BB_poly = p3d_BB_get_BB_poly_null;
  }
}

/*
 *  Get the choice of bounding box selection method
 *
 *  Output:  the method type
 */
p3d_BB_selection_type p3d_BB_get_selection_method(void) {
  return type_BB;
}

/*
 *  Choose the type of bounding box computation
 *
 *  Input:  the method type
 */
void p3d_BB_set_computation_method(p3d_BB_compute_type type) {
  switch (type) {
    case COMPUTE_BB_CLOSE:
      p3d_BB_set_mode_close();
      break;
    case COMPUTE_BB_LARGE:
      p3d_BB_set_mode_large();
      break;
    case COMPUTE_BB_COL:
      p3d_BB_set_mode_col();
      break;
  }
}

/*
 *  Get the choice of bounding box computation method
 *
 *  Output:  the method type
 */
p3d_BB_compute_type p3d_BB_get_computation_method(void) {
  return type_compute_BB;
}

void p3d_BB_set_mode_close(void) {
  p3d_BB_update_BB_obj = p3d_BB_update_BB_obj1;
  p3d_BB_get_BB_poly = p3d_BB_get_BB_poly1;
  type_compute_BB = COMPUTE_BB_CLOSE;
  /* PrintInfo(("\n BB computed by mode close\n")); */
}

void p3d_BB_set_mode_large(void) {
  p3d_BB_update_BB_obj = p3d_BB_update_BB_obj2;
  p3d_BB_get_BB_poly = p3d_BB_get_BB_poly1;
  type_compute_BB = COMPUTE_BB_LARGE;
  /* PrintInfo(("\n BB computed by mode large\n")); */
}

void p3d_BB_set_mode_col(void) {
  p3d_BB_update_BB_obj = p3d_BB_update_BB_obj1;
  p3d_BB_get_BB_poly = (void (*)(struct p3d_poly *, double *, double *, double *, double *, double *, double *))p3d_col_get_col_BB_poly_fct();
  type_compute_BB = COMPUTE_BB_COL;
  /* PrintInfo(("\n BB computed by mode col\n")); */
}

/********************************************/
/* Fonction ne faisant rien (BB d�sactiv�)  */
/* In : le polyhedre                        */
/* Out :                                    */
/********************************************/
static void p3d_BB_get_BB_poly_null(p3d_poly *p, double *x1, double *x2, double *y1, double *y2, double *z1, double *z2) { }

/********************************************/
/* Fonction ne faisant rien (BB d�sactiv�)  */
/* In : l'objet, une matrice mat quelconque */
/* Out :                                    */
/********************************************/
static void p3d_BB_update_BB_obj_null(p3d_obj *obj, p3d_matrix4 mat) { }

/********************************************/
/* Fonction recuperant la BB d'un polyhedre */
/* In : le polyhedre                        */
/* Out :                                    */
/********************************************/
void p3d_BB_get_BB_poly1(p3d_poly *p, double *x1, double *x2, double *y1, double *y2, double *z1, double *z2) {
  p3d_matrix4 mat;
  double u[3], v[3], w[6][3];
  int i, j;

  u[0] = p->box.x1;
  v[0] = p->box.x2;
  u[1] = p->box.y1;
  v[1] = p->box.y2;
  u[2] = p->box.z1;
  v[2] = p->box.z2;
  p3d_get_poly_pos(p->poly, mat);
  for (j = 0;j < 3;j++)
    for (i = 0;i < 3;i++) {
      if (mat[j][i] > 0) {
        w[j][i] = u[i];
        w[3+j][i] = v[i];
      } else {
        w[j][i] = v[i];
        w[3+j][i] = u[i];
      }
    }
  u[0] = mat[0][0] * w[0][0] + mat[0][1] * w[0][1] + mat[0][2] * w[0][2] + mat[0][3];
  u[1] = mat[1][0] * w[1][0] + mat[1][1] * w[1][1] + mat[1][2] * w[1][2] + mat[1][3];
  u[2] = mat[2][0] * w[2][0] + mat[2][1] * w[2][1] + mat[2][2] * w[2][2] + mat[2][3];
  v[0] = mat[0][0] * w[3][0] + mat[0][1] * w[3][1] + mat[0][2] * w[3][2] + mat[0][3];
  v[1] = mat[1][0] * w[4][0] + mat[1][1] * w[4][1] + mat[1][2] * w[4][2] + mat[1][3];
  v[2] = mat[2][0] * w[5][0] + mat[2][1] * w[5][1] + mat[2][2] * w[5][2] + mat[2][3];

  *x1 = u[0] - (v[0] - u[0]) * 0.025;
  *y1 = u[1] - (v[1] - u[1]) * 0.025;
  *z1 = u[2] - (v[2] - u[2]) * 0.025;
  *x2 = v[0] + (v[0] - u[0]) * 0.025;
  *y2 = v[1] + (v[1] - u[1]) * 0.025;
  *z2 = v[2] + (v[2] - u[2]) * 0.025;

  /*   PrintInfo(("polyP : [%f %f] [%f %f] [%f %f]\n",*x1,*x2,*y1,*y2,*z1,*z2)); */

}


/********************************************/
/* Fonction calculant la BB d'un objet      */
/* In : l'objet, une matrice mat quelconque */
/* Out :                                    */
/********************************************/
void p3d_BB_update_BB_obj1(p3d_obj *obj, p3d_matrix4 mat) {
  int np, i;
  double x1, x2, y1, y2, z1, z2;
  p3d_BB *BB;

  np = obj->np;
  BB = &(obj->BB);

  BB->xmin = P3D_HUGE;
  BB->xmax = -P3D_HUGE;
  BB->ymin = P3D_HUGE;
  BB->ymax = -P3D_HUGE;
  BB->zmin = P3D_HUGE;
  BB->zmax = -P3D_HUGE;

  for (i = 0;i < np;i++) {
    if (obj->pol[i]->TYPE != P3D_GRAPHIC) { // Modification Fabien
      p3d_BB_get_BB_poly(obj->pol[i], &x1, &x2, &y1, &y2, &z1, &z2);
      if (x1 < BB->xmin) {
        BB->xmin = x1;
      }
      if (x2 > BB->xmax) {
        BB->xmax = x2;
      }
      if (y1 < BB->ymin) {
        BB->ymin = y1;
      }
      if (y2 > BB->ymax) {
        BB->ymax = y2;
      }
      if (z1 < BB->zmin) {
        BB->zmin = z1;
      }
      if (z2 > BB->zmax) {
        BB->zmax = z2;
      }
    }
  }
  /*PrintInfo(("\n%sBB : [%f %f] [%f %f] [%f %f]\n",obj->name,BB->xmin,BB->xmax,BB->ymin,BB->ymax,BB->zmin,BB->zmax));*/
}


void p3d_BB_init_BB0() {
  int i, j;

  PrintInfo(("\nInitialisation des BB dans le repere propre de l objet...\n"));
  for (i = 0;i < XYZ_ENV->no;i++)         /*pour tous les obstacles */
    init_BB0_obj(XYZ_ENV->o[i]);
  for (i = 0;i < XYZ_ENV->nr;i++)         /*pour tous les robots */
    for (j = 0;j < XYZ_ENV->robot[i]->no;j++)   /*pour tous les objets du robot i */
      init_BB0_obj(XYZ_ENV->robot[i]->o[j]);
  PrintInfo(("Initialisation des BB dans le repere propre de l objet, termin�.\n"));
}


/***************************************************/
/* Fonction calculant la BB d'un objet a partir de */
/*  sa BB dans le repere propre. Cette derniere    */
/*  peut etre plus large que pour update_BB_obj    */
/* In : l'objet                                    */
/* Out :                                           */
/***************************************************/
void p3d_BB_update_BB_obj2(p3d_obj *obj, p3d_matrix4 mat) {
  p3d_BB *BB, *BB0;
  p3d_matrix4 inv_pos0, tmp;
  int i, j, np;
  double u[3], v[3], w[6][3];

  np = obj->np;
  BB = &(obj->BB);
  BB0 = &(obj->BB0);

  BB->xmin = P3D_HUGE;
  BB->xmax = -P3D_HUGE;
  BB->ymin = P3D_HUGE;
  BB->ymax = -P3D_HUGE;
  BB->zmin = P3D_HUGE;
  BB->zmax = -P3D_HUGE;

  u[0] = BB0->xmin;
  v[0] = BB0->xmax;
  u[1] = BB0->ymin;
  v[1] = BB0->ymax;
  u[2] = BB0->zmin;
  v[2] = BB0->zmax;
  if (obj->jnt != NULL) {
    p3d_matInvertXform(obj->jnt->pos0, inv_pos0);
    p3d_matMultXform(mat, inv_pos0, tmp);
    p3d_mat4Copy(tmp, mat);
  }
  for (j = 0;j < 3;j++) {
    for (i = 0;i < 3;i++) {
      if (mat[j][i] > 0) {
        w[j][i] = u[i];
        w[3+j][i] = v[i];
      } else {
        w[j][i] = v[i];
        w[3+j][i] = u[i];
      }
    }
  }
  BB->xmin = mat[0][0] * w[0][0] + mat[0][1] * w[0][1] + mat[0][2] * w[0][2] + mat[0][3];
  BB->ymin = mat[1][0] * w[1][0] + mat[1][1] * w[1][1] + mat[1][2] * w[1][2] + mat[1][3];
  BB->zmin = mat[2][0] * w[2][0] + mat[2][1] * w[2][1] + mat[2][2] * w[2][2] + mat[2][3];
  BB->xmax = mat[0][0] * w[3][0] + mat[0][1] * w[3][1] + mat[0][2] * w[3][2] + mat[0][3];
  BB->ymax = mat[1][0] * w[4][0] + mat[1][1] * w[4][1] + mat[1][2] * w[4][2] + mat[1][3];
  BB->zmax = mat[2][0] * w[5][0] + mat[2][1] * w[5][1] + mat[2][2] * w[5][2] + mat[2][3];

  /*    PrintInfo(("\n%sBB2 : [%f %f] [%f %f] [%f %f]\n",obj->name,u[0],v[0],u[1],v[1],u[2],v[2]));   */
}


/*****************************************/
/* Fonction recuperant la BB d'un robot  */
/* In : le robot                         */
/* Out :                                 */
/*****************************************/
void p3d_BB_update_BB_rob(p3d_rob *rob) {
  int no, i;
  p3d_BB *BB, *BBo;

  no = rob->no;
  BB = &(rob->BB);

  BB->xmin = P3D_HUGE;
  BB->xmax = -P3D_HUGE;
  BB->ymin = P3D_HUGE;
  BB->ymax = -P3D_HUGE;
  BB->zmin = P3D_HUGE;
  BB->zmax = -P3D_HUGE;

  for (i = 0;i < no;i++) {
    BBo = &(rob->o[i]->BB);
    if (BBo->xmin < BB->xmin) {
      BB->xmin = BBo->xmin;
    }
    if (BBo->xmax > BB->xmax) {
      BB->xmax = BBo->xmax;
    }
    if (BBo->ymin < BB->ymin) {
      BB->ymin = BBo->ymin;
    }
    if (BBo->ymax > BB->ymax) {
      BB->ymax = BBo->ymax;
    }
    if (BBo->zmin < BB->zmin) {
      BB->zmin = BBo->zmin;
    }
    if (BBo->zmax > BB->zmax) {
      BB->zmax = BBo->zmax;
    }
  }
  /* PrintInfo(("%s : [%f %f] [%f %f] [%f %f]\n",rob->name,BB->xmin,BB->xmax,BB->ymin,BB->ymax,BB->zmin,BB->zmax)); */
}

/**************************************************************/
/* Fonction testant les boites englobantes de deux objets     */
/* In : les deux objets                                       */
/* Out :                                                      */
/**************************************************************/
int p3d_BB_overlap_obj_obj(p3d_obj *obj1, p3d_obj *obj2) {
  double o1min[3], o1max[3], o2min[3], o2max[3];

  /* PrintInfo(("%s et %s \n",obj1->name,obj2->name));  */

  o1min[0] = obj1->BB.xmin;
  o1max[0] = obj1->BB.xmax;
  o1min[1] = obj1->BB.ymin;
  o1max[1] = obj1->BB.ymax;
  o1min[2] = obj1->BB.zmin;
  o1max[2] = obj1->BB.zmax;

  o2min[0] = obj2->BB.xmin;
  o2max[0] = obj2->BB.xmax;
  o2min[1] = obj2->BB.ymin;
  o2max[1] = obj2->BB.ymax;
  o2min[2] = obj2->BB.zmin;
  o2max[2] = obj2->BB.zmax;

  /*   PrintInfo(("axe 1 : [%f %f] [%f %f]\n",o1min[0],o1max[0],o2min[0],o2max[0])); */
  /*   PrintInfo(("axe 2 : [%f %f] [%f %f]\n",o1min[1],o1max[1],o2min[1],o2max[1])); */
  /*   PrintInfo(("axe 3 : [%f %f] [%f %f]\n",o1min[2],o1max[2],o2min[2],o2max[2])); */


  if ((o1max[0] <= o2min[0] || o1min[0] >= o2max[0]) ||
      (o1max[1] <= o2min[1] || o1min[1] >= o2max[1]) ||
      (o1max[2] <= o2min[2] || o1min[2] >= o2max[2])) {
    return(0);
  } else {
    return(1);
  }
}

/**************************************************************/
/* Fonction testant les boites englobantes d'un robot et d'un */
/* objet                                                      */
/* In : l'objet et le robot                                   */
/* Out :                                                      */
/**************************************************************/
int p3d_BB_overlap_rob_obj(p3d_rob *rob, p3d_obj *obj) {
  double rmin[3], rmax[3], omin[3], omax[3];

  rmin[0] = rob->BB.xmin;
  rmax[0] = rob->BB.xmax;
  rmin[1] = rob->BB.ymin;
  rmax[1] = rob->BB.ymax;
  rmin[2] = rob->BB.zmin;
  rmax[2] = rob->BB.zmax;

  omin[0] = obj->BB.xmin;
  omax[0] = obj->BB.xmax;
  omin[1] = obj->BB.ymin;
  omax[1] = obj->BB.ymax;
  omin[2] = obj->BB.zmin;
  omax[2] = obj->BB.zmax;

  /*   PrintInfo(("axe 1 : [%f %f] [%f %f]\n",rmin[0],rmax[0],omin[0],omax[0])); */
  /*   PrintInfo(("axe 2 : [%f %f] [%f %f]\n",rmin[1],rmax[1],omin[1],omax[1])); */
  /*   PrintInfo(("axe 3 : [%f %f] [%f %f]\n",rmin[2],rmax[2],omin[2],omax[2])); */

  if ((rmax[0] <= omin[0] || rmin[0] >= omax[0]) ||
      (rmax[1] <= omin[1] || rmin[1] >= omax[1]) ||
      (rmax[2] <= omin[2] || rmin[2] >= omax[2])) {
    return(0);
  } else {
    return(1);
  }
}


/**************************************************************/
/* Fonction testant les boites englobantes de deux robots     */
/* In : les deux robots                                       */
/* Out :                                                      */
/**************************************************************/
int p3d_BB_overlap_rob_rob(p3d_rob *rob1, p3d_rob *rob2) {
  double r1min[3], r1max[3], r2min[3], r2max[3];

  r1min[0] = rob1->BB.xmin;
  r1max[0] = rob1->BB.xmax;
  r1min[1] = rob1->BB.ymin;
  r1max[1] = rob1->BB.ymax;
  r1min[2] = rob1->BB.zmin;
  r1max[2] = rob1->BB.zmax;

  r2min[0] = rob2->BB.xmin;
  r2max[0] = rob2->BB.xmax;
  r2min[1] = rob2->BB.ymin;
  r2max[1] = rob2->BB.ymax;
  r2min[2] = rob2->BB.zmin;
  r2max[2] = rob2->BB.zmax;

  /*   PrintInfo(("axe 1 : [%f %f] [%f %f]\n",r1min[0],r1max[0],r2min[0],r2max[0])); */
  /*   PrintInfo(("axe 2 : [%f %f] [%f %f]\n",r1min[1],r1max[1],r2min[1],r2max[1])); */
  /*   PrintInfo(("axe 3 : [%f %f] [%f %f]\n",r1min[2],r1max[2],r2min[2],r2max[2])); */


  if ((r1max[0] <= r2min[0] || r1min[0] >= r2max[0]) ||
      (r1max[1] <= r2min[1] || r1min[1] >= r2max[1]) ||
      (r1max[2] <= r2min[2] || r1min[2] >= r2max[2])) {
    return(0);
  } else {
    return(1);
  }

}


/*************************************************/
/* Fonction calculant la distance la plus petite */
/* entre les BB de deux objets                   */
/* In : les objets                               */
/* Out : la distance                             */
/*************************************************/
double p3d_BB_obj_obj_extern_dist(p3d_obj *obj1, p3d_obj *obj2, double *dist_ut) {
  int cx = 0, cy = 0, cz = 0;
  double o1min[3], o1max[3], o2min[3], o2max[3], d;

  o1min[0] = obj1->BB.xmin;
  o1max[0] = obj1->BB.xmax;
  o1min[1] = obj1->BB.ymin;
  o1max[1] = obj1->BB.ymax;
  o1min[2] = obj1->BB.zmin;
  o1max[2] = obj1->BB.zmax;

  o2min[0] = obj2->BB.xmin;
  o2max[0] = obj2->BB.xmax;
  o2min[1] = obj2->BB.ymin;
  o2max[1] = obj2->BB.ymax;
  o2min[2] = obj2->BB.zmin;
  o2max[2] = obj2->BB.zmax;

  /*  PrintInfo(("axe 1 : [%f %f] [%f %f]\n",o1min[0],o1max[0],o2min[0],o2max[0])); */
  /*  PrintInfo(("axe 2 : [%f %f] [%f %f]\n",o1min[1],o1max[1],o2min[1],o2max[1])); */
  /*  PrintInfo(("axe 3 : [%f %f] [%f %f]\n",o1min[2],o1max[2],o2min[2],o2max[2]));  */

  if ((o1max[0] >= o2min[0] && o1min[0] <= o2max[0]) || (o1min[0] <= o2max[0] && o2min[0] <= o1max[0])) {
    cx = 1;
  }
  if ((o1max[1] >= o2min[1] && o1min[1] <= o2max[1]) || (o1min[1] <= o2max[1] && o2min[1] <= o1max[1])) {
    cy = 1;
  }
  if ((o1max[2] >= o2min[2] && o1min[2] <= o2max[2]) || (o1min[2] <= o2max[2] && o2min[2] <= o1max[2])) {
    cz = 1;
  }
  /* PrintInfo(("cx : %d, cy : %d, cz : %d\n",cx,cy,cz)); */

  /* les boites se chevauchent */
  if (cx && cy && cz) {
    d = 0.;
    if (o1max[0] >= o2min[0] && o1min[0] <= o2max[0]) {
      *dist_ut = SQR(o2min[0] - o1max[0]);
    } else {
      *dist_ut = SQR(o1min[0] - o2max[0]);
    }
    if (o1max[1] >= o2min[1] && o1min[1] <= o2max[1]) {
      *dist_ut = *dist_ut + SQR(o2min[1] - o1max[1]);
    } else {
      *dist_ut = *dist_ut + SQR(o1min[1] - o2max[1]);
    }
    if (o1max[2] >= o2min[2] && o1min[2] <= o2max[2]) {
      *dist_ut = *dist_ut + SQR(o2min[2] - o1max[2]);
    } else {
      *dist_ut = *dist_ut + SQR(o1min[2] - o2max[2]);
    }
    *dist_ut = sqrt(*dist_ut);
  }
  /* deux faces se chevauchent en projection */
  else if (cx && cy && !cz) {
    if (o1max[2] <= o2min[2] && o1min[2] <= o2max[2]) {
      d = fabs(o2min[2] - o1max[2]);
    } else {
      d = fabs(o1min[2] - o2max[2]);
    }
    if (o1max[0] >= o2min[0] && o1min[0] <= o2max[0]) {
      *dist_ut = SQR(o2min[0] - o1max[0]);
    } else {
      *dist_ut = SQR(o1min[0] - o2max[0]);
    }
    if (o1max[1] >= o2min[1] && o1min[1] <= o2max[1]) {
      *dist_ut = *dist_ut + SQR(o2min[1] - o1max[1]);
    } else {
      *dist_ut = *dist_ut + SQR(o1min[1] - o2max[1]);
    }
    *dist_ut = sqrt(*dist_ut);
  } else if (cx && cz && !cy) {
    if (o1max[1] <= o2min[1] && o1min[1] <= o2max[1]) {
      d = fabs(o2min[1] - o1max[1]);
    } else {
      d = fabs(o1min[1] - o2max[1]);
    }
    if (o1max[0] >= o2min[0] && o1min[0] <= o2max[0]) {
      *dist_ut = SQR(o2min[0] - o1max[0]);
    } else {
      *dist_ut = SQR(o1min[0] - o2max[0]);
    }
    if (o1max[2] >= o2min[2] && o1min[2] <= o2max[2]) {
      *dist_ut = *dist_ut + SQR(o2min[2] - o1max[2]);
    } else {
      *dist_ut = *dist_ut + SQR(o1min[2] - o2max[2]);
    }
    *dist_ut = sqrt(*dist_ut);
  } else if (cy && cz && !cx) {
    if (o1max[0] <= o2min[0] && o1min[0] <= o2max[0]) {
      d = fabs(o2min[0] - o1max[0]);
    } else {
      d = fabs(o1min[0] - o2max[0]);
    }
    if (o1max[1] >= o2min[1] && o1min[1] <= o2max[1]) {
      *dist_ut = SQR(o2min[1] - o1max[1]);
    } else {
      *dist_ut = SQR(o1min[1] - o2max[1]);
    }
    if (o1max[2] >= o2min[2] && o1min[2] <= o2max[2]) {
      *dist_ut = *dist_ut + SQR(o2min[2] - o1max[2]);
    } else {
      *dist_ut = *dist_ut + SQR(o1min[2] - o2max[2]);
    }
    *dist_ut = sqrt(*dist_ut);
  }
  /* deux aretes se chevauchent en projection */
  else if (cx && !cy && !cz) {
    if (o1max[1] <= o2min[1] && o1min[1] <= o2max[1]) {
      d = SQR(o2min[1] - o1max[1]);
    } else {
      d = SQR(o1min[1] - o2max[1]);
    }
    if (o1max[2] <= o2min[2] && o1min[2] <= o2max[2]) {
      d = d + SQR(o1max[2] - o2min[2]);
    } else {
      d = d + SQR(o1min[2] - o2max[2]);
    }
    d = sqrt(d);
    if (o1max[0] <= o2min[0] && o1min[0] <= o2max[0]) {
      *dist_ut = fabs(o2min[0] - o1max[0]);
    } else {
      *dist_ut = fabs(o1min[0] - o2max[0]);
    }
  } else if (cy && !cx && !cz) {
    if (o1max[0] <= o2min[0] && o1min[0] <= o2max[0]) {
      d = SQR(o1max[0] - o2min[0]);
    } else {
      d = SQR(o2max[0] - o1min[0]);
    }
    if (o1max[2] <= o2min[2] && o1min[2] <= o2max[2]) {
      d = d + SQR(o1max[2] - o2min[2]);
    } else {
      d = d + SQR(o2max[2] - o1min[2]);
    }
    d = sqrt(d);
    if (o1max[1] <= o2min[1] && o1min[1] <= o2max[1]) {
      *dist_ut = fabs(o2min[1] - o1max[1]);
    } else {
      *dist_ut = fabs(o1min[1] - o2max[1]);
    }
  } else if (cz && !cx && !cy) {
    if (o1max[0] <= o2min[0] && o1min[0] <= o2max[0]) {
      d = SQR(o1max[0] - o2min[0]);
    } else {
      d = SQR(o2max[0] - o1min[0]);
    }
    if (o1max[1] <= o2min[1] && o1min[1] <= o2max[1]) {
      d = d + SQR(o1max[1] - o2min[1]);
    } else {
      d = d + SQR(o2max[1] - o1min[1]);
    }
    d = sqrt(d);
    if (o1max[2] <= o2min[2] && o1min[2] <= o2max[2]) {
      *dist_ut = fabs(o2min[2] - o1max[2]);
    } else {
      *dist_ut = fabs(o1min[2] - o2max[2]);
    }
  }
  /* aucun chevauchement meme en projection */
  else {
    if (o1max[0] <= o2min[0] && o1min[0] <= o2max[0]) {
      d = SQR(o1max[0] - o2min[0]);
    } else {
      d = SQR(o2max[0] - o1min[0]);
    }
    if (o1max[1] <= o2min[1] && o1min[1] <= o2max[1]) {
      d = d + SQR(o1max[1] - o2min[1]);
    } else {
      d = d + SQR(o2max[1] - o1min[1]);
    }
    if (o1max[2] <= o2min[2] && o1min[2] <= o2max[2]) {
      d = d + SQR(o1max[2] - o2min[2]);
    } else {
      d = d + SQR(o2max[2] - o1min[2]);
    }
    d = sqrt(d);
    *dist_ut = d;
  }
  /* PrintInfo(("distance ext %s/%s : %f %f\n",obj1->name,obj2->name,d,*dist_ut)); */

  return(d);
}




/*****************************************/
/* Fonction calculant la BB d'un objet   */
/* In : l'objet                          */
/* Out :                                 */
/*****************************************/
static void init_BB0_obj(p3d_obj *obj) {
  int i, j, k, np;
  p3d_BB *BB0;
  double x1, y1, z1, x2, y2, z2;
  p3d_matrix4 mat;
  double u[3], v[3], w[6][3];

  np = obj->np;
  BB0 = &(obj->BB0);

  BB0->xmin = P3D_HUGE;
  BB0->xmax = -P3D_HUGE;
  BB0->ymin = P3D_HUGE;
  BB0->ymax = -P3D_HUGE;
  BB0->zmin = P3D_HUGE;
  BB0->zmax = -P3D_HUGE;

  for (i = 0;i < np;i++) {
    if (obj->pol[i]->TYPE != P3D_GRAPHIC) {  // Modification Fabien
      for (j = 0;j < 4;j++)
        for (k = 0;k < 4;k++)
          mat[j][k] = obj->pol[i]->pos0[j][k];

      u[0] = obj->pol[i]->box.x1;
      v[0] = obj->pol[i]->box.x2;
      u[1] = obj->pol[i]->box.y1;
      v[1] = obj->pol[i]->box.y2;
      u[2] = obj->pol[i]->box.z1;
      v[2] = obj->pol[i]->box.z2;
      for (j = 0;j < 3;j++)
        for (k = 0;k < 3;k++) {
          if (mat[j][k] > 0) {
            w[j][k] = u[k];
            w[3+j][k] = v[k];
          } else {
            w[j][k] = v[k];
            w[3+j][k] = u[k];
          }
        }
      u[0] = mat[0][0] * w[0][0] + mat[0][1] * w[0][1] + mat[0][2] * w[0][2] + mat[0][3];
      u[1] = mat[1][0] * w[1][0] + mat[1][1] * w[1][1] + mat[1][2] * w[1][2] + mat[1][3];
      u[2] = mat[2][0] * w[2][0] + mat[2][1] * w[2][1] + mat[2][2] * w[2][2] + mat[2][3];
      v[0] = mat[0][0] * w[3][0] + mat[0][1] * w[3][1] + mat[0][2] * w[3][2] + mat[0][3];
      v[1] = mat[1][0] * w[4][0] + mat[1][1] * w[4][1] + mat[1][2] * w[4][2] + mat[1][3];
      v[2] = mat[2][0] * w[5][0] + mat[2][1] * w[5][1] + mat[2][2] * w[5][2] + mat[2][3];

      x1 = u[0] - (v[0] - u[0]) * 0.025;
      y1 = u[1] - (v[1] - u[1]) * 0.025;
      z1 = u[2] - (v[2] - u[2]) * 0.025;
      x2 = v[0] + (v[0] - u[0]) * 0.025;
      y2 = v[1] + (v[1] - u[1]) * 0.025;
      z2 = v[2] + (v[2] - u[2]) * 0.025;

      /* p3d_get_BB_poly(obj->pol[i],&x1,&x2,&y1,&y2,&z1,&z2); */
      if (x1 < BB0->xmin) {
        BB0->xmin = x1;
      }
      if (x2 > BB0->xmax) {
        BB0->xmax = x2;
      }
      if (y1 < BB0->ymin) {
        BB0->ymin = y1;
      }
      if (y2 > BB0->ymax) {
        BB0->ymax = y2;
      }
      if (z1 < BB0->zmin) {
        BB0->zmin = z1;
      }
      if (z2 > BB0->zmax) {
        BB0->zmax = z2;
      }
    }
  }
  /*  PrintInfo(("\n%sBB0 : [%f %f] [%f %f] [%f %f]\n",obj->name,BB0->xmin,BB0->xmax,BB0->ymin,BB0->ymax,BB0->zmin,BB0->zmax)); */
}


/*--------------------------------------------------------------------------
 *--------------------------------------------------------------------------
 * p3d_BB_handle Creator / Destructor
 */


/*--------------------------------------------------------------------------*/
/*! \brief Create a new handle for the current environment.
 *
 * \return The new handle of collision (::NULL if there is an error).
 *
 * \note No collision is activate.
 */
p3d_BB_handle * p3d_BB_handle_create(void) {
  p3d_BB_handle * handlePt;
  int i;

  handlePt = MY_ALLOC(p3d_BB_handle, 1);
  if (handlePt == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  handlePt->env_id = XYZ_ENV->num;
  handlePt->nb_robot = XYZ_ENV->nr;
  handlePt->lists_links_autocol = MY_ALLOC(p3d_elem_list_BB *,
                                  handlePt->nb_robot);
  handlePt->lists_links_rob = MY_ALLOC(p3d_elem_list_BB *, handlePt->nb_robot);
  handlePt->lists_links_env = MY_ALLOC(p3d_elem_list_BB *, handlePt->nb_robot);
  if ((handlePt->lists_links_autocol == NULL) ||
      (handlePt->lists_links_rob == NULL) ||
      (handlePt->lists_links_env == NULL)) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  for (i = 0; i < handlePt->nb_robot; i++) {
    handlePt->lists_links_autocol[i] = NULL;
    handlePt->lists_links_rob[i]     = NULL;
    handlePt->lists_links_env[i]     = NULL;
  }
  return handlePt;
}


/*--------------------------------------------------------------------------*/
/*! \brief This function deactivates the computation of distances
 *         between all the object's bounding boxes in the list.
 *
 * \param  first_elem: A pointer on the first element of the list.
 *
 * \internal
 */
static void s_p3d_BB_deactivate_list(p3d_elem_list_BB ** first_elem) {
  p3d_elem_list_BB * new_elem, * tmp_elem;

  new_elem = *first_elem;
  while (new_elem != NULL) {
    tmp_elem = new_elem;
    new_elem = new_elem->next;
    MY_FREE(tmp_elem, p3d_elem_list_BB, 1);
  }
  *first_elem = NULL;
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy a handle for bounding box collisions.
 *
 * \param The handle (::NULL to destroy the current handle).
 *
 * \note This function cannot destroy the default environment.
 *       To do that use p3d_BB_clear().
 */
void p3d_BB_handle_destroy(p3d_BB_handle * handlePt) {
  int i;

  if (handlePt == NULL) {
    handlePt = cur_BB_handlePt;
  }
  if ((handlePt != default_BB_handlePt) && (handlePt != NULL)) {
    for (i = 0; i < handlePt->nb_robot; i++) {
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_autocol[i]));
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_rob[i]));
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_env[i]));
    }
    MY_FREE(handlePt->lists_links_autocol, p3d_elem_list_BB*,
            handlePt->nb_robot);
    MY_FREE(handlePt->lists_links_rob, p3d_elem_list_BB*, handlePt->nb_robot);
    MY_FREE(handlePt->lists_links_env, p3d_elem_list_BB*, handlePt->nb_robot);
    MY_FREE(handlePt, p3d_BB_handle, 1);
  }
  if (handlePt == cur_BB_handlePt) {
    cur_BB_handlePt = default_BB_handlePt;
  }
}



/*--------------------------------------------------------------------------
 *--------------------------------------------------------------------------
 * Current bounding box collision handle management
 */

/*--------------------------------------------------------------------------*/
/*! \brief Clean all variables use for the management of
 *         bounding box collision handle.
 */
void p3d_BB_clear(void) {
  int i;

  if (default_BB_handlePt != NULL) {
    for (i = 0; i < default_BB_handlePt->nb_robot; i++) {
      s_p3d_BB_deactivate_list(&(default_BB_handlePt->lists_links_autocol[i]));
      s_p3d_BB_deactivate_list(&(default_BB_handlePt->lists_links_rob[i]));
      s_p3d_BB_deactivate_list(&(default_BB_handlePt->lists_links_env[i]));
    }
    MY_FREE(default_BB_handlePt->lists_links_autocol, p3d_elem_list_BB*,
            default_BB_handlePt->nb_robot);
    MY_FREE(default_BB_handlePt->lists_links_rob, p3d_elem_list_BB*,
            default_BB_handlePt->nb_robot);
    MY_FREE(default_BB_handlePt->lists_links_env, p3d_elem_list_BB*,
            default_BB_handlePt->nb_robot);
    MY_FREE(default_BB_handlePt, p3d_BB_handle, 1);
    default_BB_handlePt = NULL;
  }
  cur_BB_handlePt = NULL;
}


/*--------------------------------------------------------------------------*/
/*! \brief Select a new bounding box collision handle.
 *
 * \param  handlePt: The bounding box collision handle table.
 */
void p3d_BB_set_cur_handle(p3d_BB_handle * handlePt) {
  if (handlePt != NULL) {
    cur_BB_handlePt = handlePt;
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the current bounding box collision handle.
 *
 * \return The current bounding box collision handle table.
 */
p3d_BB_handle * p3d_BB_get_cur_handle(void) {
  return cur_BB_handlePt;
}


/*--------------------------------------------------------------------------*/
/*! \brief Select a valid current bounding box collision handler.
 *
 * \note Check if the current environment is initialized.
 * \note Check if the current environment is in the right environment.
 * \internal
 */
static void s_p3d_BB_sel_valid_cur_BB_handle(void) {
  if ((cur_BB_handlePt == NULL) || (cur_BB_handlePt->env_id != XYZ_ENV->num)) {
    if ((default_BB_handlePt != NULL) &&
        (default_BB_handlePt->env_id != XYZ_ENV->num)) {
      p3d_BB_clear();  /* Destroy it */
    }
    if (default_BB_handlePt == NULL) {
      default_BB_handlePt = p3d_BB_handle_create();  /* Create it */
    }
    cur_BB_handlePt = default_BB_handlePt;
  }
}


/*--------------------------------------------------------------------------
 *--------------------------------------------------------------------------
 * Activation / Deactivation
 */

/*--------------------------------------------------------------------------*/
/*! \brief Add a collision pair between two objects in the same robot.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \note Check if this pair already exist.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 and \a obj2 must be store by the same robot.
 * \warning We must have: \a obj1->jnt->num < \a obj2->jnt->num.
 * \internal
 */
static void s_p3d_BB_activate_autocol(p3d_BB_handle * handlePt,
                                      p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_autocol[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_autocol[obj1->jnt->rob->num];
  while ((next_elem != NULL) && ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
                                 ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
                                  (next_elem->obj2->jnt->num < obj2->jnt->num)))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem == NULL) || (next_elem->obj1 != obj1) ||
      (next_elem->obj2 != obj2)) { /* Do not already exist */
    new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
    if (new_elem != NULL) {
      new_elem->obj1 = obj1;
      new_elem->obj2 = obj2;
      new_elem->next = next_elem;
      (*prev_elem) = new_elem;
    } else {
      PrintError(("Not enough memory !!!\n"));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Add a collision pair between two objects in two different robot.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \note Activate only the pair for the robot of \a obj1.
 * \note Check if this pair already exist.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 and \a obj2 must be store by different robots.
 * \internal
 */
static void s_p3d_BB_activate_rob(p3d_BB_handle * handlePt,
                                  p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_rob[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_rob[obj1->jnt->rob->num];
  while ((next_elem != NULL) &&
         ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
          ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
           ((next_elem->obj2->jnt->rob->num < obj2->jnt->rob->num) ||
            ((next_elem->obj2->jnt->rob->num == obj2->jnt->rob->num) &&
             (next_elem->obj2->jnt->num < obj2->jnt->num)))))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem == NULL) || (next_elem->obj1 != obj1) ||
      (next_elem->obj2 != obj2)) { /* Do not already exist */
    new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
    if (new_elem != NULL) {
      new_elem->obj1 = obj1;
      new_elem->obj2 = obj2;
      new_elem->next = next_elem;
      (*prev_elem) = new_elem;
    } else {
      PrintError(("Not enough memory !!!\n"));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Add a collision pair between a body and an object
 *         in the environment.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The body.
 * \param  obj2:     The environment object.
 *
 * \note Check if this pair already exist.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 must be store by a robot.
 * \warning \a obj2 must be store by the environment.
 * \internal
 */
static void s_p3d_BB_activate_env(p3d_BB_handle * handlePt,
                                  p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_env[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_env[obj1->jnt->rob->num];
  while ((next_elem != NULL) && ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
                                 ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
                                  (next_elem->obj2->num < obj2->num)))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem == NULL) || (next_elem->obj1 != obj1) ||
      (next_elem->obj2 != obj2)) { /* Do not already exist */
    new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
    if (new_elem != NULL) {
      new_elem->obj1 = obj1;
      new_elem->obj2 = obj2;
      new_elem->next = next_elem;
      (*prev_elem) = new_elem;
    } else {
      PrintError(("Not enough memory !!!\n"));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function activates the computation of
 *          distances between the two object's bounding boxes.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \note Check if this pair already exist.
 * \note If \a handlePt == ::NULL then use the current handle.
 */
void p3d_BB_activate_pair(p3d_BB_handle * handlePt,
                          p3d_obj *obj1, p3d_obj *obj2) {
  if (type_BB != DEACTIVATE_BB) {
    if (handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      handlePt = cur_BB_handlePt;
    }
    if (handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (p3d_col_object_is_pure_graphic(obj1) ||
        p3d_col_object_is_pure_graphic(obj2)) {
      return;
    }
    if ((obj1->jnt != NULL) && (obj1->jnt->rob != NULL)) {
      if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
        if (obj1->jnt->rob == obj2->jnt->rob) { /* Autocollision */
          if (obj1->jnt->num < obj2->jnt->num) {
            s_p3d_BB_activate_autocol(handlePt, obj1, obj2);
          } else if (obj1->jnt->num > obj2->jnt->num) {
            s_p3d_BB_activate_autocol(handlePt, obj2, obj1);
          }
        } else { /* Links between two robots */
          s_p3d_BB_activate_rob(handlePt, obj1, obj2);
          s_p3d_BB_activate_rob(handlePt, obj2, obj1);
        }
      } else if (obj2->jnt == NULL) { /* Links with the environment */
        s_p3d_BB_activate_env(handlePt, obj1, obj2);
      }
    } else if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL) &&
               (obj1->jnt == NULL)) {
      s_p3d_BB_activate_env(handlePt, obj2, obj1);
    }
  }
}


/*
 * add obst to list_BB for all robots in the environment
 */
void p3d_BB_activate(p3d_obj *obst) {
  int nof_robots;
  int nof_bodies;
  int i, j;
  p3d_obj *body;

  nof_robots = XYZ_ENV->nr;
  for (i = 0;i < nof_robots;i++) {
    nof_bodies = XYZ_ENV->robot[i]->no;
    for (j = 0;j < nof_bodies;j++) {
      body = XYZ_ENV->robot[i]->o[j];
      p3d_BB_activate_pair((p3d_BB_handle *)NULL, body, obst);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Deactivate a collision pair between two objects in the same robot.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 and \a obj2 must be store by the same robot.
 * \warning We must have: \a obj1->jnt->num < \a obj2->jnt->num.
 * \internal
 */
static void s_p3d_BB_deactivate_autocol(p3d_BB_handle * handlePt,
                                        p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_autocol[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_autocol[obj1->jnt->rob->num];
  while ((next_elem != NULL) && ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
                                 ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
                                  (next_elem->obj2->jnt->num < obj2->jnt->num)))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem != NULL) && (next_elem->obj1 == obj1) &&
      (next_elem->obj2 == obj2)) { /* Found */
    (*prev_elem) = next_elem->next;
    MY_FREE(next_elem, p3d_elem_list_BB, 1);
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Deactivate a collision pair between two objects
 *         in two different robot.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \note Deactivate only the pair for the robot of \a obj1.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 and \a obj2 must be store by different robots.
 * \internal
 */
static void s_p3d_BB_deactivate_rob(p3d_BB_handle * handlePt,
                                    p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_rob[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_rob[obj1->jnt->rob->num];
  while ((next_elem != NULL) &&
         ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
          ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
           ((next_elem->obj2->jnt->rob->num < obj2->jnt->rob->num) ||
            ((next_elem->obj2->jnt->rob->num == obj2->jnt->rob->num) &&
             (next_elem->obj2->jnt->num < obj2->jnt->num)))))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem != NULL) && (next_elem->obj1 == obj1) &&
      (next_elem->obj2 == obj2)) { /* Found */
    (*prev_elem) = next_elem->next;
    MY_FREE(next_elem, p3d_elem_list_BB, 1);
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Deactivate a collision pair between a body and an object
 *         in the environment.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The body.
 * \param  obj2:     The environment object.
 *
 * \warning \a handlePt must be valid.
 * \warning \a obj1 must be store by a robot.
 * \warning \a obj2 must be store by the environment.
 * \internal
 */
static void s_p3d_BB_deactivate_env(p3d_BB_handle * handlePt,
                                    p3d_obj *obj1, p3d_obj *obj2) {
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(handlePt->lists_links_env[obj1->jnt->rob->num]);
  next_elem = handlePt->lists_links_env[obj1->jnt->rob->num];
  while ((next_elem != NULL) && ((next_elem->obj1->jnt->num < obj1->jnt->num) ||
                                 ((next_elem->obj1->jnt->num == obj1->jnt->num) &&
                                  (next_elem->obj2->num < obj2->num)))) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem != NULL) && (next_elem->obj1 == obj1) &&
      (next_elem->obj2 == obj2)) { /* Found */
    (*prev_elem) = next_elem->next;
    MY_FREE(next_elem, p3d_elem_list_BB, 1);
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function deactivates the computation of
 *          distances between the two object's bounding boxes.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj1:     The first object.
 * \param  obj2:     The second object.
 *
 * \note If \a handlePt == ::NULL then use the current handle.
 */
void p3d_BB_deactivate_pair(p3d_BB_handle * handlePt,
                            p3d_obj *obj1, p3d_obj *obj2) {
  if (type_BB != DEACTIVATE_BB) {
    if (handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      handlePt = cur_BB_handlePt;
    }
    if (handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if ((obj1->jnt != NULL) && (obj1->jnt->rob != NULL)) {
      if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
        if (obj1->jnt->rob == obj2->jnt->rob) { /* Autocollision */
          if (obj1->jnt->num < obj2->jnt->num) {
            s_p3d_BB_deactivate_autocol(handlePt, obj1, obj2);
          } else if (obj1->jnt->num > obj2->jnt->num) {
            s_p3d_BB_deactivate_autocol(handlePt, obj2, obj1);
          }
        } else { /* Links between two robots */
          s_p3d_BB_deactivate_rob(handlePt, obj1, obj2);
          s_p3d_BB_deactivate_rob(handlePt, obj2, obj1);
        }
      } else if (obj2->jnt == NULL) { /* Links with the environment */
        s_p3d_BB_deactivate_env(handlePt, obj1, obj2);
      }
    } else if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL) &&
               (obj1->jnt == NULL)) {
      s_p3d_BB_deactivate_env(handlePt, obj2, obj1);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function activates the computation of
 *          distances between one body and all the objects in the environment.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj:      The body.
 *
 * \note If \a handlePt == ::NULL then use the current handle.
 */
void p3d_BB_activate_env(p3d_BB_handle * handlePt, p3d_obj *obj) {
  int o, no, io;
  p3d_obj * obst;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  if (type_BB != DEACTIVATE_BB) {
    if (handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      handlePt = cur_BB_handlePt;
    }
    if (handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (p3d_col_object_is_pure_graphic(obj) || (obj->jnt == NULL) ||
        (obj->jnt->rob == NULL)) {
      return;
    }

    prev_elem = &(handlePt->lists_links_env[obj->jnt->rob->num]);
    next_elem = handlePt->lists_links_env[obj->jnt->rob->num];
    while ((next_elem != NULL) && (next_elem->obj1->jnt->num < obj->jnt->num)) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    }

    o = p3d_get_desc_curnum(P3D_OBSTACLE);
    no =  p3d_get_desc_number(P3D_OBSTACLE);
    for (io = 0; io < no; io++) {
      p3d_sel_desc_num(P3D_OBSTACLE, io);
      obst = (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);
      if (!p3d_col_object_is_pure_graphic(obst)) {
        if ((next_elem != NULL) && (next_elem->obj1 == obj) &&
            (next_elem->obj2 == obst)) { /* Already exist */
          prev_elem = &(next_elem->next);
          next_elem = next_elem->next;
        } else {
          new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
          if (new_elem != NULL) {
            new_elem->obj1 = obj;
            new_elem->obj2 = obst;
            new_elem->next = next_elem;
            (*prev_elem) = new_elem;
            prev_elem = &(new_elem->next);
          } else {
            PrintError(("Not enough memory !!!\n"));
          }
        }
      }
    }
    p3d_sel_desc_num(P3D_OBSTACLE, o);
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function deactivates the computation of
 *          distances between one body and all the objects in the environment.
 *
 * \param  handlePt: The bounding box handle collision table.
 * \param  obj:      The body.
 *
 * \note If \a handlePt == ::NULL then use the current handle.
 */
void p3d_BB_deactivate_env(p3d_BB_handle * handlePt, p3d_obj *obj) {
  p3d_elem_list_BB * tmp_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  if (type_BB != DEACTIVATE_BB) {
    if (handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      handlePt = cur_BB_handlePt;
    }
    if (handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if ((obj == NULL) || (obj->jnt == NULL) || (obj->jnt->rob == NULL)) {
      PrintError(("The object is not a body !!!\n"));
      return;
    }

    prev_elem = &(handlePt->lists_links_env[obj->jnt->rob->num]);
    next_elem = handlePt->lists_links_env[obj->jnt->rob->num];
    while ((next_elem != NULL) && (next_elem->obj1->jnt->num < obj->jnt->num)) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    }
    while ((next_elem != NULL) && (next_elem->obj1 == obj)) {
      tmp_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(tmp_elem, p3d_elem_list_BB, 1);
    }
  }
}


/*--------------------------------------------------------------------------
 *--------------------------------------------------------------------------
 * Bounding box collision handle management.
 */

/*--------------------------------------------------------------------------*/
/*! \brief  This function deactivates all the computation of
 *          distances for all robots.
 *
 * \param  handlePt: The bounding box handle collision table.
 *
 * \note If \a handlePt == ::NULL then use the current handle.
 * \note After this function the distances computed are HUGE_VAL.
 */
void p3d_BB_deactivate_all(p3d_BB_handle * handlePt) {
  int i;

  if (type_BB != DEACTIVATE_BB) {
    if (handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      handlePt = cur_BB_handlePt;
    }
    if (handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    for (i = 0; i < handlePt->nb_robot; i++) {
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_autocol[i]));
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_rob[i]));
      s_p3d_BB_deactivate_list(&(handlePt->lists_links_env[i]));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Copy the list of autocollision into \a dest_handlePt for the robot
 *         specified.
 *
 * \param  src_handlePt:  The bounding box handle collision table
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the copy.
 * \param  i_robot:       The indice of the robot.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_copy_into_autocol(p3d_BB_handle * src_handlePt,
                                       p3d_BB_handle * dest_handlePt,
                                       int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_autocol[i_robot]);
  next_elem = dest_handlePt->lists_links_autocol[i_robot];
  src_elem = src_handlePt->lists_links_autocol[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Copy the list of collision with other robot into
 *         \a dest_handlePt for the robot specified.
 *
 * \param  src_handlePt:  The bounding box handle collision table
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the copy.
 * \param  i_robot:       The indice of the robot.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_copy_into_rob(p3d_BB_handle * src_handlePt,
                                   p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_rob[i_robot]);
  next_elem = dest_handlePt->lists_links_rob[i_robot];
  src_elem = src_handlePt->lists_links_rob[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob == next_elem->obj2->jnt->rob) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          ((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob->num > next_elem->obj2->jnt->rob->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Copy the list of collision with the environment into
 *         \a dest_handlePt for the robot specified.
 *
 * \param  src_handlePt:  The bounding box handle collision table
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the copy.
 * \param  i_robot:       The indice of the robot.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_copy_into_env(p3d_BB_handle * src_handlePt,
                                   p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_env[i_robot]);
  next_elem = dest_handlePt->lists_links_env[i_robot];
  src_elem = src_handlePt->lists_links_env[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->num > next_elem->obj2->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function copy \a src_handlePt a bounding box
 *          collision handle into \a dest_handlePt.
 *
 * \param  src_handlePt:  The bounding box collision handle
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the copy.
 *
 * \note If \a src_handlePt == ::NULL then use the current handle.
 * \note If \a dest_handlePt == ::NULL then use the current handle.
 */
void p3d_BB_handle_copy_into(p3d_BB_handle * src_handlePt,
                             p3d_BB_handle * dest_handlePt) {
  int i;

  if (type_BB != DEACTIVATE_BB) {
    if (src_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      src_handlePt = cur_BB_handlePt;
    }
    if (src_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (dest_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      dest_handlePt = cur_BB_handlePt;
    }
    if (dest_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (src_handlePt != dest_handlePt) {
      for (i = 0; i < src_handlePt->nb_robot; i++) {
        s_p3d_BB_copy_into_autocol(src_handlePt, dest_handlePt, i);
        s_p3d_BB_copy_into_rob(src_handlePt, dest_handlePt, i);
        s_p3d_BB_copy_into_env(src_handlePt, dest_handlePt, i);
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief  This function copy \a src_handlePt a bounding box collision handle.
 *
 * \param  src_handlePt:  The bounding box collision handle
 *                        that must be copied.
 *
 * \return The copy of the bounding box collision handle.
 *
 * \note If \a src_handlePt == ::NULL then use the current handle.
 */
p3d_BB_handle * p3d_BB_handle_copy(p3d_BB_handle * src_handlePt) {
  p3d_BB_handle * dest_handlePt;

  dest_handlePt = p3d_BB_handle_create();
  p3d_BB_handle_copy_into(src_handlePt, dest_handlePt);
  return dest_handlePt;
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two list of autocollision into \a dest_handlePt for the robot
 *         specified (sum of collision tests).
 *
 * \param  src_handlePt:  The source bounding box handle collision table.
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the sum.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "or" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_add_into_autocol(p3d_BB_handle * src_handlePt,
                                      p3d_BB_handle *dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_autocol[i_robot]);
  next_elem = dest_handlePt->lists_links_autocol[i_robot];
  src_elem = src_handlePt->lists_links_autocol[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two list of collision with other robot into
 *         \a dest_handlePt for the robot specified (sum of collision tests).
 *
 * \param  src_handlePt:  The source bounding box handle collision table.
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the sum.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "or" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_add_into_rob(p3d_BB_handle * src_handlePt,
                                  p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_rob[i_robot]);
  next_elem = dest_handlePt->lists_links_rob[i_robot];
  src_elem = src_handlePt->lists_links_rob[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob == next_elem->obj2->jnt->rob) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          ((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob->num > next_elem->obj2->jnt->rob->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two lists of collision with the environment into
 *         \a dest_handlePt for the robot specified (sum of collision tests).
 *
 * \param  src_handlePt:  The source bounding box handle collision table.
 *                        that must be copied.
 * \param  dest_handlePt: The bounding box handle collision table
 *                        that must store the sum.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "or" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_add_into_env(p3d_BB_handle * src_handlePt,
                                  p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_env[i_robot]);
  next_elem = dest_handlePt->lists_links_env[i_robot];
  src_elem = src_handlePt->lists_links_env[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->num > next_elem->obj2->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      new_elem = MY_ALLOC(p3d_elem_list_BB, 1);
      if (new_elem != NULL) {
        new_elem->obj1 = src_elem->obj1;
        new_elem->obj2 = src_elem->obj2;
        new_elem->next = next_elem;
        (*prev_elem) = new_elem;
        prev_elem = &(new_elem->next);
      } else {
        PrintError(("Not enough memory !!!\n"));
      }
      src_elem = src_elem->next;
    }
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Merge two bounding box collision handle (sum of collision tests)
 *
 * \param  src_handlePt:  The source bounding box collision handle.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the sum.
 *
 * It is like an "or" opperation between all pair.
 *
 * \note If \a src_handlePt == ::NULL then use the current handle.
 * \note If \a dest_handlePt == ::NULL then use the current handle.
 */
void p3d_BB_handle_add_into(p3d_BB_handle * src_handlePt,
                            p3d_BB_handle * dest_handlePt) {
  int i;

  if (type_BB != DEACTIVATE_BB) {
    if (src_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      src_handlePt = cur_BB_handlePt;
    }
    if (src_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (dest_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      dest_handlePt = cur_BB_handlePt;
    }
    if (dest_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (src_handlePt != dest_handlePt) {
      for (i = 0; i < src_handlePt->nb_robot; i++) {
        s_p3d_BB_add_into_autocol(src_handlePt, dest_handlePt, i);
        s_p3d_BB_add_into_rob(src_handlePt, dest_handlePt, i);
        s_p3d_BB_add_into_env(src_handlePt, dest_handlePt, i);
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two lists of autocollision into \a dest_handlePt
 *         for the robot specified (substraction of collision tests).
 *
 * \param  src_handlePt:  The source bounding box collision handle.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the substraction.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "dest and not src" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_sub_into_autocol(p3d_BB_handle * src_handlePt,
                                      p3d_BB_handle *dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_autocol[i_robot]);
  next_elem = dest_handlePt->lists_links_autocol[i_robot];
  src_elem = src_handlePt->lists_links_autocol[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two lists of collision with other robot into
 *         \a dest_handlePt for the robot specified
 *         (substraction of collision tests).
 *
 * \param  src_handlePt:  The source bounding box collision handle.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the substraction.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "dest and not src" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_sub_into_rob(p3d_BB_handle * src_handlePt,
                                  p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_rob[i_robot]);
  next_elem = dest_handlePt->lists_links_rob[i_robot];
  src_elem = src_handlePt->lists_links_rob[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob == next_elem->obj2->jnt->rob) &&
           (src_elem->obj2->jnt->num > next_elem->obj2->jnt->num)) ||
          ((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->jnt->rob->num > next_elem->obj2->jnt->rob->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      src_elem = src_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Merge two lists of collision with the environment into
 *         \a dest_handlePt for the robot specified
 *         (substraction of collision tests).
 *
 * \param  src_handlePt:  The source bounding box collision handle.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the substraction.
 * \param  i_robot:       The indice of the robot.
 *
 * It is like an "dest and not src" opperation between all pair.
 *
 * \warning \a src_handlePt and \a dest_handlePt must be valid.
 * \internal
 */
static void s_p3d_BB_sub_into_env(p3d_BB_handle * src_handlePt,
                                  p3d_BB_handle * dest_handlePt, int i_robot) {
  p3d_elem_list_BB * src_elem;
  p3d_elem_list_BB * new_elem;
  p3d_elem_list_BB ** prev_elem;
  p3d_elem_list_BB * next_elem;

  prev_elem = &(dest_handlePt->lists_links_env[i_robot]);
  next_elem = dest_handlePt->lists_links_env[i_robot];
  src_elem = src_handlePt->lists_links_env[i_robot];
  while ((src_elem != NULL) || (next_elem != NULL)) {
    if ((src_elem == NULL) ||
        ((next_elem != NULL) &&
         (((src_elem->obj1 == next_elem->obj1) &&
           (src_elem->obj2->num > next_elem->obj2->num)) ||
          (src_elem->obj1->jnt->num > next_elem->obj1->jnt->num)))) {
      new_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(new_elem, p3d_elem_list_BB, 1);
    } else if ((next_elem != NULL) && (src_elem != NULL) &&
               (next_elem->obj1 == src_elem->obj1) &&
               (next_elem->obj2 == src_elem->obj2)) {
      src_elem = src_elem->next;
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    } else {
      src_elem = src_elem->next;
    }
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Merge two bounding box collision handle
 *        (substraction of collision tests)
 *
 * \param  src_handlePt:  The source bounding box collision handle.
 * \param  dest_handlePt: The bounding box collision handle
 *                        that must store the substraction.
 *
 * It is like an "dest and not src" opperation between all pair.
 *
 * \note If \a src_handlePt == ::NULL then use the current handle.
 * \note If \a dest_handlePt == ::NULL then use the current handle.
 */
void p3d_BB_handle_sub_into(p3d_BB_handle * src_handlePt,
                            p3d_BB_handle * dest_handlePt) {
  int i;

  if (type_BB != DEACTIVATE_BB) {
    if (src_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      src_handlePt = cur_BB_handlePt;
    }
    if (src_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (dest_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
      dest_handlePt = cur_BB_handlePt;
    }
    if (dest_handlePt->env_id != XYZ_ENV->num) {
      PrintError(("Handle not valid!!!\n"));
      return;
    }
    if (src_handlePt != dest_handlePt) {
      for (i = 0; i < src_handlePt->nb_robot; i++) {
        s_p3d_BB_sub_into_autocol(src_handlePt, dest_handlePt, i);
        s_p3d_BB_sub_into_rob(src_handlePt, dest_handlePt, i);
        s_p3d_BB_sub_into_env(src_handlePt, dest_handlePt, i);
      }
    }
  }
}



/****************************************************************************
 ***************************************************************************
 * Multi-environment management for bounding box.
 */


/*--------------------------------------------------------------------------*/
/*! \brief Store the global variable in ::BB_envs.
 *
 *  \internal
 */
static void s_p3d_deconnect_BB_global_vars(void) {
  if (index_cur_BB_envs >= 0) {
    BB_envs[index_cur_BB_envs].cur_BB_handlePt = cur_BB_handlePt;
    cur_BB_handlePt = NULL;
    BB_envs[index_cur_BB_envs].default_BB_handlePt = default_BB_handlePt;
    default_BB_handlePt = NULL;
  } else {
    p3d_BB_clear();
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Restor the data from an environment backup.
 *
 *  \param BB_envPt: Data for bounding box selection
 *                   in a specific environment.
 *
 *  \internal
 */
static void s_p3d_connect_BB_global_vars(p3d_BB_env * BB_envPt) {
  cur_BB_handlePt = BB_envPt->cur_BB_handlePt;
  default_BB_handlePt = BB_envPt->default_BB_handlePt;
}


/*--------------------------------------------------------------------------*/
/*! \brief Create a new case to store the new envirenment \a p3d_env_id.
 *
 *  \param p3d_env_id: Number of the environment.
 *
 *  \internal
 */
static void s_p3d_init_BB_global_vars(int p3d_env_id) {
  BB_envs = MY_REALLOC(BB_envs, p3d_BB_env, nof_BB_envs, nof_BB_envs + 1);
  if (BB_envs == NULL) {
    PrintError(("Not enough memory !!!\n"));
    nof_BB_envs = 0;
    index_cur_BB_envs = -1;
  } else {
    index_cur_BB_envs = nof_BB_envs;
    nof_BB_envs ++;
    BB_envs[index_cur_BB_envs].p3d_env_id = p3d_env_id;
    BB_envs[index_cur_BB_envs].cur_BB_handlePt = NULL;
    BB_envs[index_cur_BB_envs].default_BB_handlePt = NULL;
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Initialize collision pair to start a new environment.
 *
 * This function must be used each time there is a new environment, or
 * each time that the environment switches.
 */
void p3d_BB_start(void) {
  int p3d_env_id;
  int i;

  s_p3d_deconnect_BB_global_vars();
  p3d_env_id = XYZ_ENV->num;
  for (i = 0; i < nof_BB_envs; i++) {
    if (BB_envs[i].p3d_env_id == p3d_env_id) {
      break;
    }
  }
  if ((nof_BB_envs > i) && (BB_envs[i].p3d_env_id == p3d_env_id)) {
    /* Already stored */
    s_p3d_connect_BB_global_vars(&(BB_envs[i]));
  } else {
    /* Create this new environment */
    s_p3d_init_BB_global_vars(p3d_env_id);
  }
}


/*--------------------------------------------------------------------------*/
/*!  \brief Release the memory for collision pair in all environment.
 *
 * This function must be used at the end of the application
 * to release the memory.
 */
void p3d_BB_stop(void) {
  int i;

  s_p3d_deconnect_BB_global_vars();
  for (i = 0; i < nof_BB_envs; i++) {
    s_p3d_connect_BB_global_vars(&(BB_envs[i]));
    p3d_col_pair_clear();
  }
  MY_FREE(BB_envs, p3d_BB_env, nof_BB_envs);
  nof_BB_envs = 0;
  index_cur_BB_envs = -1;
  BB_envs = NULL;
}


/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*! \brief This function compute the distances between the bounding boxes
 *         for the robot specified.
 *
 * \param  robotPt:   The robot
 *
 * \retval distances: The array of distances.
 */
void p3d_BB_dist_robot(p3d_rob *robotPt, double *distances) {
  p3d_elem_list_BB * elem;
  int i, njnt;
  double dist, dist_ut;

  njnt = robotPt->njoints;

  if (type_BB == DEACTIVATE_BB) {
    for (i = 0; i <= njnt; i++) {
      distances[i] = 0.0;
    }
  } else {
    for (i = 0; i <= njnt; i++) {
      distances[i] = P3D_HUGE;
    }

    if (cur_BB_handlePt == NULL) {
      s_p3d_BB_sel_valid_cur_BB_handle();
    }

    elem = cur_BB_handlePt->lists_links_autocol[robotPt->num];
    while (elem != NULL) {
      dist = p3d_BB_obj_obj_extern_dist(elem->obj1, elem->obj2, &dist_ut);
      i = elem->obj1->jnt->num;
      if (distances[i] > dist) {
        distances[i] = dist;
      }
      i = elem->obj2->jnt->num;
      if (distances[i] > dist) {
        distances[i] = dist;
      }
      elem = elem->next;
    }
    elem = cur_BB_handlePt->lists_links_rob[robotPt->num];
    while (elem != NULL) {
      dist = p3d_BB_obj_obj_extern_dist(elem->obj1, elem->obj2, &dist_ut);
      i = elem->obj1->jnt->num;
      if (distances[i] > dist) {
        distances[i] = dist;
      }
      elem = elem->next;
    }
    elem = cur_BB_handlePt->lists_links_env[robotPt->num];
    while (elem != NULL) {
      dist = p3d_BB_obj_obj_extern_dist(elem->obj1, elem->obj2, &dist_ut);
      i = elem->obj1->jnt->num;
      if (distances[i] > dist) {
        distances[i] = dist;
      }
      elem = elem->next;
    }
  }
}
