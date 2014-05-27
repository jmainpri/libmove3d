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
/* (jcortes) */
/****************************************************************************/
/*! \file cntrt.h
 *
 *  \brief constraints, RLG, and parallel structure data
 *
 ****************************************************************************/

#include "p3d.h"

#ifndef CNTRT_H
#define CNTRT_H

#define DOF_WITHOUT_CNTRT 0
#define DOF_PASSIF 2
#define DOF_ACTIF  1

/**********************
    PARALLEL SYS
***********************/

/*-----------------------------------------------------------------------------*/
/*! \brief This structure characterizes the triangle associated to three attachmets
 */
typedef struct s_tri_attach {
  /*! \brief Transformations from the frane associated to the triangle
   *         to the frame of the parallel platform (grasped object)
   */
  p3d_matrix4          Ttri2pp;
  int                  index_att[3];
  double               l12;
  double               l13;
  double               l23;
  double               angle12;
  struct s_tri_attach  *next;
} p3d_tri_attach;


/*-----------------------------------------------------------------------------*/
/*! \brief This structure identifies the different parts of the parallel system
 */
#define MAX_N_MANIPULATORS 10
typedef struct s_parallel {
  /*! \brief The base (reference) jnt */
  struct jnt            *base_jntPt;
  /*! \brief The parallel platform (grasped object) jnt */
  struct jnt            *platform_jntPt;
  /*! \brief  Function used to generate de platform configuration */
  int (*fct_parplatf_shoot)(struct s_parallel*, configPt);
  /*! \brief The parallel platform (grasped object) jnt new rand-bounds */
  double                platform_dof_min_bounds[6];
  double                platform_dof_max_bounds[6];
  /*! \brief The number of manipulators */
  int                   n_manipulators;
  /*! \brief The list of attachments */
  struct jnt            *att_jnt[MAX_N_MANIPULATORS];
  /*! \brief Index for link_between_joint platform-attach */
  int                   index_link_platf[MAX_N_MANIPULATORS];
  /*! \brief The distance between the att and the platform frame */
  double                dpa[MAX_N_MANIPULATORS];
  /*! \brief The list of cntrts (one by manipulator) */
  struct cntrt          *manip_cntrt[MAX_N_MANIPULATORS];
  /*! \brief The list of triplets of attachmets
   *         (required by Juan Cortes's method to bound the parallel platform dofs)
   */
  struct s_tri_attach   *triangles;
} p3d_parallel;


/**********************
        RLG
***********************/

/* clasement of joints for RLG */
#define RJA1 1
#define RJA2 2
#define RJB1 3
#define RJB2 4
#define RJB3 5
#define RJC0 6
#define RJC1 7
#define RJD1 8
#define RJD2 9
#define TJ   10

typedef struct s_shell {
  double rext;
  double rint;
  double alpha;
} p3d_shell;

typedef struct s_rlg_chain_data {
  struct jnt  *jnt;
  int          num_dof_jnt;
  int          jtype;
  double       lmax;
  double       lmin;
  double       dat[4];
  struct s_shell shell;
  double       totrml;
  double       totmMdif;
  p3d_matrix4  Tref;
  double       refval;
} p3d_rlg_chain_data, *pp3d_rlg_chain_data;

typedef struct s_rlg_chain {
  int                      nlinksrlgch;
  struct s_rlg_chain_data  **rlg_data;
  int (*rlg_chain_fct)(struct cntrt* ct, configPt q);
} p3d_rlg_chain, *pp3d_rlg_chain;


typedef struct s_rlg_base {
  struct jnt               *basejntPt;
  int (*rlg_base_fct)(struct cntrt*, configPt);
  double                   rmax, rmin;
  p3d_matrix4              Tbm;
  double                   lbm;
} p3d_rlg_base, *pp3d_rlg_base;



/*-----------------------------------------------------------------------------*/
/*! \brief Structure for RLG data
 */

typedef struct s_rlg {
  struct s_rlg_chain  *rlgchPt;
  struct s_rlg_base   *rlgbsPt;
} p3d_rlg, *pp3d_rlg;

/**********************
      CONSTRAINTS
***********************/

#define MAX_ARGU_CNTRT 30

/*! \brief Structure containing data of a constraint */
typedef struct cntrt {
  int            num;
  char           namecntrt[40];
  int            active;
  int (*fct_cntrt)(struct cntrt *ct, int iksol, configPt, double dl);
  int            nactjnts, npasjnts;
  int            ndval, nival;
  struct jnt *   actjnts[MAX_ARGU_CNTRT];
  int            act_jnt_dof[MAX_ARGU_CNTRT];
  int            act_rob_dof[MAX_ARGU_CNTRT];
  int            actjnt_state[MAX_ARGU_CNTRT];
  struct jnt *   pasjnts[MAX_ARGU_CNTRT];
  int            pas_jnt_dof[MAX_ARGU_CNTRT];
  int            pas_rob_dof[MAX_ARGU_CNTRT];
  int            argu_i[MAX_ARGU_CNTRT];
  double         argu_d[MAX_ARGU_CNTRT];

  int            markedForSingularity;//this singularity is marked for singularity don't compute the cntrt        0
  int            nSingularities;//number of singularities
  struct singularity *singularities[MAX_ARGU_CNTRT];//array of singularities

  p3d_matrix4    Tatt;
  p3d_matrix4    Tatt2;
  p3d_matrix4	 Tatt_default;
  p3d_matrix4    Tbase;
  p3d_matrix4    TSingularity;
  struct obj *   col_pairs[2][MAX_ARGU_CNTRT];
  struct cntrt   *next_cntrt;
  struct cntrt   *prev_cntrt;
  int            enchained_rob_dof[MAX_ARGU_CNTRT];
  int            nenchained;
  struct cntrt   **enchained;
  struct cntrt   *reshoot_ct;
  int            nctud;
  struct cntrt   **ct_to_update;
  struct s_rlg      *rlgPt;
  struct s_parallel *parallel_sys_data;
  struct s_bio_6R_ik_data *bio_ik_data;
  struct cntrt_management *cntrt_manager;
  int           nbSol; //Number of possible solutions. This parameter is filled in the set_function of the constraint.
} p3d_cntrt, *pp3d_cntrt;

/*! \brief Structure of singular joint value
 */
typedef struct singJntVal {
  int jntNum; //The joint number
  double *val; //The joint singular value
} p3d_singJntVal, *pp3d_singJntVal;


/*! \brief Structure of singularity
 */
typedef struct singularity {
  int nJnt; //the number of jnt for this singularity
  p3d_singJntVal *singJntVal[MAX_ARGU_CNTRT]; //array of singular joint values composing this singularity
  int nRel; //the number of relation between two classes
  int classes[MAX_ARGU_CNTRT][2]; //two columns array of the classes that this singularity can connect
} p3d_singularity, *pp3d_singularity;

/*! \brief Structure to manage the gestion of constraints in a robot */
typedef struct cntrt_management {

  /*! \brief Robot's constraints */
  p3d_cntrt   **cntrts;

  /*! \brief Number of robot's constraints */
  int         ncntrts;

  /*! \brief Number of degree of freedom */
  int         nb_dof;

  /*! \brief State of the constraints on the degree of freedom
   *
   * \note without (0), activate (1), passive (2)
   */
  int         *in_cntrt;

  /*! \brief constraints list in calling order */
  struct s_dbl_list   *cntrt_call_list;

} p3d_cntrt_management, *pp3d_cntrt_management;

// struct lessJnt {
//   bool operator() (const struct jnt* jnt1, const struct jnt* jnt2) const
//   {return jnt1->num < jnt2->num;}
// };
// 
// class Constraint{
//   private:
//     int id;
//     std::string name;
//     int active;
//     std::multimap <struct jnt*, std::vector <int>, lessJnt> actjnts;
//     std::multimap <struct jnt*, std::vector <int>, lessJnt> pasjnts;
//     std::vector <int> argu_i;
//     std::vector <double> argu_d;
//     p3d_matrix4 Tatt;
//     p3d_matrix4 Tatt2;
//     std::vector <pair <struct obj*, struct obj*>> col_pairs;
// //     struct obj * col_pairs[2][MAX_ARGU_CNTRT];
// //     struct cntrt *next_cntrt;
// //     struct cntrt *prev_cntrt;
//     int enchained_rob_dof[MAX_ARGU_CNTRT];
//     int nenchained;
//     std::vector <Constraint&> enchained;
//     Constraint& reshoot_ct;
//     int nctud;
//     std::vector <Constraint&> ct_to_update;
//     struct s_rlg *rlgPt;
//     struct s_parallel *parallel_sys_data;
//     struct s_bio_6R_ik_data *bio_ik_data;
//     struct cntrt_management *cntrt_manager;
// 
//     p3d_matrix4 Tbase;
//     p3d_matrix4 TSingularity;
//     int markedForSingularity;//this singularity is marked for singularity don't compute the cntrt 0
//     int nbSol;
//     int nSingularities;//number of singularities
//     std::vector<struct singularity* > singularities;//vector of singularities
// };

#endif

