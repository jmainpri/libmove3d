/* ---------------------------------------------------------------------*/
/*! \file group.h
 * \brief defined the sub part of robots used eg. PA10 ....
 *
 * \author X; Broquere
 * \date   Sept. 2008
 */
#ifndef __P3D_LOCALPATHGROUP_H__
#define __P3D_LOCALPATHGROUP_H__


#include"../other_libraries/gbM/src/gbStruct.h"


typedef enum {
 BASE,
 FREEFLYER,
 JOINT
} p3d_group_type;

typedef struct gp_freeflyer_params{
	double   J_max_lin;   /* Initialized with p3d files */
	double   A_max_lin;   /* Initialized with p3d files */
	double   V_max_lin;   /* Initialized with p3d files */
	double   J_max_ang;   /* Initialized with p3d files */
	double   A_max_ang;   /* Initialized with p3d files */
	double   V_max_ang;   /* Initialized with p3d files */
} gp_freeflyer_params_str, *pgp_freeflyer_str;


typedef struct gp_joint_params{
  int     nbJoint;         /* number of joints in the group */
	double  * J_max;   /* Initialized with p3d files */
	double  * A_max;   /* Initialized with p3d files */
	double  * V_max;   /* Initialized with p3d files */
} gp_joint_str, *pgp_joint_str;

/* pointeur sur les structues specifiques a chaque groupe */
typedef union gp_specific {
  pgp_freeflyer_str freeflyer_params;
  pgp_joint_str   joint_params;
} p3d_gp_specific, *pp3d_gp_specific;

#endif
