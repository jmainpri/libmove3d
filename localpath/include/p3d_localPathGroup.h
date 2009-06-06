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
 PA10_ARM,
 KUKA_ARM,
 JOINT
} p3d_group_type;

typedef struct gp_pa10Arm_params{
	Gb_6rParameters pa10;
	Gb_th R6RT;
	Gb_th invR6RT;
	double   J_max_lin;   /* Initialized with p3d files */
	double   A_max_lin;   /* Initialized with p3d files */
	double   V_max_lin;   /* Initialized with p3d files */
	double   J_max_ang;   /* Initialized with p3d files */
	double   A_max_ang;   /* Initialized with p3d files */
	double   V_max_ang;   /* Initialized with p3d files */
} gp_pa10Arm_str, *pgp_pa10Arm_str;


typedef struct gp_kukaArm_params{
  double   J_max_lin;   /* Initialized with p3d files */
	double   A_max_lin;   /* Initialized with p3d files */
	double   V_max_lin;   /* Initialized with p3d files */
	double   J_max_ang;   /* Initialized with p3d files */
	double   A_max_ang;   /* Initialized with p3d files */
	double   V_max_ang;   /* Initialized with p3d files */
} gp_kukaArm_str, *pgp_kukaArm_str;

typedef struct gp_joint_params{
  int     nbJoint;         /* number of joints in the group */
	double  * J_max;   /* Initialized with p3d files */
	double  * A_max;   /* Initialized with p3d files */
	double  * V_max;   /* Initialized with p3d files */
} gp_joint_str, *pgp_joint_str;

/* pointeur sur les structues specifiques a chaque groupe */
typedef union gp_specific {
  pgp_pa10Arm_str pa10Arm_params;
  pgp_kukaArm_str kukaArm_params;
  pgp_joint_str   joint_params;
} p3d_gp_specific, *pp3d_gp_specific;

#endif
