/* ---------------------------------------------------------------------*/
/*! \file group.h
 * \brief defined the sub part of robots used eg. PA10 ....
 *
 * \author X; Broquere
 * \date   Sept. 2008
 */
#ifndef __P3D_LOCALPATHGROUP_H__
#define __P3D_LOCALPATHGROUP_H__

#if defined(USE_GBM)
 #include "gbM/gbStruct.h"
#endif

typedef enum {
 BASE,
 FREEFLYER,
 JOINT
} p3d_group_type;

typedef struct gp_specific_params{
	int     nbDofs;    /* number of dofs in the group */
	double  * J_max;   /* Initialized with p3d files */
	double  * A_max;   /* Initialized with p3d files */
	double  * V_max;   /* Initialized with p3d files */
} gp_specific_str, *pgp_specific_str;

#endif
