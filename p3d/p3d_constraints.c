/* (jcortes) */
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

#if defined(MULTILOCALPATH)
#include "gbM/gbStruct.h"
#endif
#ifdef LIGHT_PLANNER
  #include "lightPlannerApi.h"
  #include "ManipulationUtils.hpp"
#endif
#include <iostream>

// FOR DEBUGGING //
#define DEBUG_CNTRTS 0
///////////////////

#define sqr(x) ((x)*(x))

#define CNTRT_FIXED_NAME               "p3d_fixed_jnt"
#define CNTRT_LIN_REL_NAME             "p3d_lin_rel_dofs"
#define CNTRT_REL_NAME                 "p3d_rel_dofs"
#define CNTRT_ON_GROUND_NAME           "p3d_jnt_on_ground"
#define CNTRT_RRPR_NAME                "p3d_RRPRlnk"
#define CNTRT_4R_NAME                  "p3d_4Rlnk"
#define CNTRT_P3R_NAME                 "p3d_P3Rlnk"
#define CNTRT_3RPR_NAME                "p3d_3RPRlnk"
#define CNTRT_CAR_FRONT_WHEELS_NAME    "p3d_car_front_wheels"
#define CNTRT_CYCAB_WHEELS_NAME        "p3d_cycab_wheels"
#define CNTRT_PLANAR_CLOSED_CHAIN_NAME "p3d_planar_closed_chain"
#define CNTRT_IN_SPHERE_NAME           "p3d_in_sphere"
#define CNTRT_JNTS_RELPOS_BOUND        "p3d_jnts_relpos_bound"
#define CNTRT_FIX_JNTS_RELPOS          "p3d_fix_jnts_relpos"
#define CNTRT_3R_ARM_NAME              "p3d_3R_arm_ik"
#define CNTRT_R6_ARM_NAME              "p3d_R6_arm_ik"

#define CNTRT_PRISMATIC_ACTUATOR_NAME  "p3d_prismatic_actuator_ik"
#define CNTRT_PRISMATIC_ACTUATOR_II_NAME  "p3d_prismatic_actuator_II_ik"

#define CNTRT_6R_BIO_IK_NAME           "p3d_6R_bio_ik"
#define CNTRT_6R_BIO_IK_NAME_NOPEP     "p3d_6R_bio_ik_nopep"
#define CNTRT_6R_BIO_IK_NAME_NOPEP_NEW "p3d_6R_bio_ik_nopep_new"
#define CNTRT_BIO_BKB_HBOND            "bio_bkb_Hbond_cntrt"
#define CNTRT_BIO_DIS_BOND             "bio_diS_bond_cntrt"

#define CNTRT_CTBOT_NAME               "p3d_CTBot_ik"
//modif Mokhtar
#define CNTRT_MIN_MAX_NAME             "p3d_min_max_dofs"
#define CNTRT_KUKA_ARM_IK_NAME         "p3d_kuka_arm_ik"

#define CNTRT_R7_HUMAN_ARM_NAME        "p3d_R7_human_arm_ik"

#define CNTRT_PA10_6_ARM_IK_NAME       "p3d_pa10_6_arm_ik"
#define CNTRT_LWR_ARM_IK_NAME          "p3d_lwr_arm_ik"
#define CNTRT_PR2_ARM_IK_NAME          "p3d_pr2_arm_ik"

#define CNTRT_HEAD_OBJECT_TRACK_NAME   "p3d_head_object_track"
#include "../graphic/proto/g3d_draw_proto.h"

static int st_iksol_size = 0;
static int st_nbIkSols = 0;
static int **st_iksol = NULL;
static int *st_niksol = NULL;
static double ***st_ikSolConfig = NULL;//Array to put valids configurations of constraints
// static int look_iksol = 0;


static void p3d_set_cntrt_Tsing(p3d_cntrt *ct);
static int p3d_get_singularities_to_cross(p3d_cntrt *ct, int startSol,
                                          int goalSol, int **singList, int depth);
static int p3d_is_singularity(p3d_cntrt_management *cntrt_manager, int *ikSol);

static int p3d_set_fixed_dof(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nbVal,
                             double* val, int ct_num, int state);
static int p3d_set_lin_rel_dofs(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof, int nb_act,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nb_Dval,
                                double* Dval, int ct_num, int state);
static int p3d_set_rel_dofs(p3d_cntrt_management * cntrt_manager,
                            p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                            p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double k1,
                            double k2, double k3, double k4, int ct_num, int state);
static int p3d_set_jnt_on_ground(p3d_cntrt_management * cntrt_manager,
                                 int nb_pas, p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                 p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nb_Dval,
                                 double* Dval, int *Ival, int ct_num, int state);
static int p3d_set_4Rlnk(p3d_cntrt_management * cntrt_manager,
                         p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                         p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                         int state);
static int p3d_set_RRPRlnk(p3d_cntrt_management * cntrt_manager,
                           p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                           p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                           int state);
static int p3d_set_P3Rlnk(p3d_cntrt_management * cntrt_manager, /* << modif EF pour Delmia */
                          p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof, p3d_jnt **act_jntPt,
                          int *act_jnt_dof, int *act_rob_dof, int ct_num, int state);
static int p3d_set_3RPRlnk(p3d_cntrt_management * cntrt_manager,
                           p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                           p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                           int state);

static int p3d_set_car_front_wheels(p3d_cntrt_management * cntrt_manager,
                                    p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                    p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double h,
                                    double a, int ct_num, int state);
static int p3d_set_cycab_wheels(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double l1,
                                double l2, double e, int ct_num, int state);
static int p3d_set_planar_closed_chain(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int whatcase,
                                       int ct_num, int state);

static int p3d_fct_fixed_jnt(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int
p3d_fct_lin_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_jnt_on_ground(p3d_cntrt *ct, int iksol, configPt qp,
                                 double dl);
static int p3d_fct_4Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_RRPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_P3Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl); /* << modif EF pour Delmia */
static int p3d_fct_3RPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_car_front_wheels(p3d_cntrt *ct, int iksol, configPt qp,
                                    double dl);
static int
p3d_fct_cycab_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_planar_closed_chain(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl);

/* -- functions for RLG test -- */
static int p3d_set_in_sphere(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double x,
                             double y, double z, int ct_num, int state);
static int p3d_fct_in_sphere(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */
static int p3d_set_jnts_relpos_bound(p3d_cntrt_management * cntrt_manager,
                                     p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                     p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                     int *Ival, int ct_num, int state);
static int p3d_fct_jnts_relpos_bound(p3d_cntrt *ct, int iksol, configPt qp,
                                     double dl);

/* -- functions for cntrt fixing the distance range of 2 jnts relative position -- */
static int p3d_set_fix_jnts_relpos(p3d_cntrt_management * cntrt_manager,
                                   p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                   int *Ival, int ct_num, int state);
static int p3d_fct_fix_jnts_relpos(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl);

/* -- functions for 3R IK -- */
static int p3d_set_3R_arm_ik(p3d_cntrt_management * cntrt_manager, int nb_pas,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                             int *Ival, int ct_num, int state);
static int p3d_fct_3R_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for R6 IK -- */
static int p3d_set_R6_arm_ik(p3d_cntrt_management * cntrt_manager, int nb_pas,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                             int *Ival, int ct_num, int state);
static int p3d_fct_R6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for prismatic actuator IK -- */
static int p3d_set_prismatic_actuator_ik(p3d_cntrt_management * cntrt_manager,
                                         p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                         p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                         int ct_num, int state);
static int p3d_fct_prismatic_actuator_ik(p3d_cntrt *ct, int iksol, configPt qp,
                                         double dl);

static int p3d_set_prismatic_actuator_II_ik(
                                            p3d_cntrt_management * cntrt_manager, p3d_jnt **pas_jntPt,
                                            int *pas_jnt_dof, int *pas_rob_dof, p3d_jnt **act_jntPt,
                                            int *act_jnt_dof, int *act_rob_dof, double l0, int ct_num, int state);
static int p3d_fct_prismatic_actuator_II_ik(p3d_cntrt *ct, int iksol,
                                            configPt qp, double dl);

/* -- functions for 6R_bio_ik -- */
static int p3d_set_6R_bio_ik(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                             int state);
static int p3d_fct_6R_bio_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_6R_bio_ik_nopep(p3d_cntrt_management * cntrt_manager,
                                   p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                                   int state);
static int p3d_fct_6R_bio_ik_nopep(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl);

static int p3d_set_6R_bio_ik_nopep_new(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                       int ct_num, int state);
static int p3d_fct_6R_bio_ik_nopep_new(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl);

static int p3d_set_bio_bkb_Hbond_cntrt(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                       int *Ival, int ct_num, int state);
static int p3d_fct_bio_bkb_Hbond_cntrt(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl);

static int p3d_set_bio_diS_bond_cntrt(p3d_cntrt_management * cntrt_manager,
                                      p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                      p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                      int *Ival, int ct_num, int state);
static int p3d_fct_bio_diS_bond_cntrt(p3d_cntrt *ct, int iksol, configPt qp,
                                      double dl);

/* -- functions for CTBot IK -- */
static int p3d_set_CTBot_ik(p3d_cntrt_management * cntrt_manager,
                            p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                            p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                            int ct_num, int state);
static int p3d_fct_CTBot_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/*modif Mokhtar*/
static int p3d_set_min_max_dofs(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double k,
                                double C, int ct_num, int state);
static int
p3d_fct_min_max_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for KUKA 7 DoF arm IK -- */
static int p3d_set_kuka_arm_ik(p3d_cntrt_management * cntrt_manager,
                               p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                               p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                               double * dVal, int ct_num, int state);
static int
p3d_fct_kuka_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

#if defined(USE_GBM)

/* -- functions for LWR arm IK -- */
static int p3d_set_lwr_arm_ik(p3d_cntrt_management * cntrt_manager,
                              p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                              p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                              double * dVal, int ct_num, int state);
static int p3d_fct_lwr_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for LWR arm IK -- */
static int p3d_set_pr2_arm_ik(p3d_cntrt_management * cntrt_manager,
                              p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                              p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                              double * dVal, int ct_num, int state);
static int p3d_fct_pr2_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

#endif

static int p3d_set_R7_human_arm_ik(p3d_cntrt_management * cntrt_manager,
                                   int nb_pas, p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                   int *Ival, int ct_num, int state);
static int p3d_fct_R7_human_arm_ik(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl);

#if defined(USE_GBM)
/* -- functions for PA10-6 DoF arm IK -- */
static int p3d_set_pa10_6_arm_ik(p3d_cntrt_management * cntrt_manager,
                                 p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                 p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                                 double * dVal, int ct_num, int state);
static int p3d_fct_pa10_6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp,
                                 double dl);
#endif

static int p3d_set_head_object_track(p3d_cntrt_management * cntrt_manager,
                                     p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                     p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                                     double * dVal, int ct_num, int state);
static int p3d_fct_head_object_track(p3d_cntrt *ct, int iksol, configPt qp,
                                     double dl);

/*fmodif Mokhtar*/
static p3d_cntrt *last_cntrt_set = NULL;

/*---------------------------------------------------------------------------*/
static int TEST_PHASE = 0;

void p3d_set_TEST_PHASE(int val) {
	TEST_PHASE = val;
}

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*! \brief Funtion that returns the number of parameters needed for
 *         each type of constraints.
 *
 * \param  namecntrt: Name of the constraints
 *
 * \retval nb_Dofpasiv: Number of passive degree of freedom.
 * \retval nb_Dofactiv: Number of active degree of freedom.
 * \retval nb_Dval:     Number of float parameters.
 * \retval nb_Ival:     Number of integer parameters.
 *
 * \note If the constraints doesn't exist then all numbers are set to 0.
 */
void p3d_constraint_get_nb_param(const char *namecntrt, int *nb_Dofpasiv,
                                 int *nb_Dofactiv, int *nb_Dval, int *nb_Ival) {
	*nb_Dofpasiv = 0;
	*nb_Dofactiv = 0;
	*nb_Dval = 0;
	*nb_Ival = 0;
  
	if (strcmp(namecntrt, CNTRT_FIXED_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dval = 1;
	} else if (strcmp(namecntrt, CNTRT_LIN_REL_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 2;
	} else if (strcmp(namecntrt, CNTRT_REL_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 2;
		*nb_Dval = 4;
	} else if (strcmp(namecntrt, CNTRT_ON_GROUND_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dval = 2;
		*nb_Ival = 3;
	} else if (strcmp(namecntrt, CNTRT_RRPR_NAME) == 0) {
		*nb_Dofpasiv = 2;
		*nb_Dofactiv = 1;
	} else if (strcmp(namecntrt, CNTRT_4R_NAME) == 0) {
		*nb_Dofpasiv = 3;
		*nb_Dofactiv = 1;
	} else if (strcmp(namecntrt, CNTRT_P3R_NAME) == 0) {
		*nb_Dofpasiv = 2;
		*nb_Dofactiv = 1;
	} else if (strcmp(namecntrt, CNTRT_3RPR_NAME) == 0) {
		*nb_Dofpasiv = 2;
		*nb_Dofactiv = 2;
	} else if (strcmp(namecntrt, CNTRT_CAR_FRONT_WHEELS_NAME) == 0) {
		*nb_Dofpasiv = 2;
		*nb_Dofactiv = 1;
		*nb_Dval = 2;
	} else if (strcmp(namecntrt, CNTRT_CYCAB_WHEELS_NAME) == 0) {
		*nb_Dofpasiv = 4;
		*nb_Dofactiv = 1;
		*nb_Dval = 3;
	} else if (strcmp(namecntrt, CNTRT_PLANAR_CLOSED_CHAIN_NAME) == 0) {
		*nb_Dofpasiv = 4;
		*nb_Dofactiv = 1;
		*nb_Ival = 1;
	} else if (strcmp(namecntrt, CNTRT_IN_SPHERE_NAME) == 0) {
		*nb_Dval = 3;
	} else if (strcmp(namecntrt, CNTRT_JNTS_RELPOS_BOUND) == 0) {
		*nb_Dofpasiv = 0;
		*nb_Dofactiv = 0;
		*nb_Dval = 2;
		*nb_Ival = 2;
	} else if (strcmp(namecntrt, CNTRT_FIX_JNTS_RELPOS) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 0;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_3R_ARM_NAME) == 0) {
		*nb_Dofpasiv = 3;
		*nb_Dofactiv = 1;
		*nb_Dval = 3;
		*nb_Ival = 1;
	} else if (strcmp(namecntrt, CNTRT_R6_ARM_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 3;
		*nb_Ival = 1;
	} else if (strcmp(namecntrt, CNTRT_PRISMATIC_ACTUATOR_NAME) == 0) {
		*nb_Dofpasiv = 3;
		*nb_Dofactiv = 1;
		*nb_Dval = 3;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_PRISMATIC_ACTUATOR_II_NAME) == 0) {
		*nb_Dofpasiv = 3;
		*nb_Dofactiv = 1;
		*nb_Dval = 1;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 0;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME_NOPEP) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 0;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME_NOPEP_NEW) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 12;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_BIO_BKB_HBOND) == 0) {
		*nb_Dofpasiv = 0;
		*nb_Dofactiv = 0;
		*nb_Dval = 4;
		*nb_Ival = 4;
	} else if (strcmp(namecntrt, CNTRT_BIO_DIS_BOND) == 0) {
		*nb_Dofpasiv = 0;
		*nb_Dofactiv = 0;
		*nb_Dval = 2;
		*nb_Ival = 2;
	} else if (strcmp(namecntrt, CNTRT_CTBOT_NAME) == 0) {
		*nb_Dofpasiv = 13;
		*nb_Dofactiv = 1;
		*nb_Dval = 0;
		*nb_Ival = 0;
	}/*modif mokhtar*/
	else if (strcmp(namecntrt, CNTRT_MIN_MAX_NAME) == 0) {
		*nb_Dofpasiv = 0;
		*nb_Dofactiv = 2;
		*nb_Dval = 2;
		*nb_Ival = 0;
	} else if (strcmp(namecntrt, CNTRT_KUKA_ARM_IK_NAME) == 0) {
		*nb_Dofpasiv = 6;//1-2-4-5-6-7
		*nb_Dofactiv = 1;//freeflyerDof
		*nb_Dval = 0;
		*nb_Ival = 3;//fixed joint, left or right arm, solution number 1-8
	} else if (strcmp(namecntrt, CNTRT_LWR_ARM_IK_NAME) == 0) {
#if defined(USE_GBM)
		*nb_Dofpasiv = 6;//1-2-4-5-6-7
		*nb_Dofactiv = 1;//freeflyerDof
		*nb_Dval = 0;
		*nb_Ival = 2;//fixed joint, solution number 1-8   
#else
		printf("!!! Constraint %s not created !!! \n",CNTRT_LWR_ARM_IK_NAME);
		printf("!!! Compile with GBM library !!! \n");
#endif
	} else if (strcmp(namecntrt, CNTRT_PR2_ARM_IK_NAME) == 0) {
#if defined(USE_GBM)
		*nb_Dofpasiv = 6;//1-2-4-5-6-7
		*nb_Dofactiv = 1;//freeflyerDof
		*nb_Dval = 0;
		*nb_Ival = 2;//fixed joint, solution number 1-8   
#else
		printf("!!! Constraint %s not created !!! \n",CNTRT_PR2_ARM_IK_NAME);
		printf("!!! Compile with GBM library !!! \n");
#endif
	} else if (strcmp(namecntrt, CNTRT_R7_HUMAN_ARM_NAME) == 0) {
		*nb_Dofpasiv = 1;
		*nb_Dofactiv = 1;
		*nb_Dval = 2;
		*nb_Ival = 1;
	} else if (strcmp(namecntrt, CNTRT_PA10_6_ARM_IK_NAME) == 0) {
#if defined(USE_GBM)
		*nb_Dofpasiv = 6;//1-2-4-5-6-7
		*nb_Dofactiv = 1;//freeflyerDof
		*nb_Dval = 0;
		*nb_Ival = 1;//solution number 1-8
#else
		printf("!!! Constraint %s not created !!! \n",CNTRT_PA10_6_ARM_IK_NAME);
		printf("!!! Compile with GBM library !!! \n");
#endif
	} else if (strcmp(namecntrt, CNTRT_HEAD_OBJECT_TRACK_NAME) == 0) {
		*nb_Dofpasiv = 2;
		*nb_Dofactiv = 1;//object Dof
		*nb_Dval = 0;
		*nb_Ival = 0;
	}/*fmodif mokhtar*/
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to create a new constraints into a constraints manager.
 *
 * \param cntrt_manager: The constraint manager.
 * \param namecntrt:     The name of the constraints.
 * \param nb_pas:        The number of passive degrees of freedom.
 * \param pas_jntPt:     The array of passive joints.
 * \param pas_jnt_dof:   The array of passive degree of freedom in the joints
 * \param Dofpassiv:     The array of passive degree of freedom in a robot.
 * \param nb_act:        The number of active degrees of freedom.
 * \param act_jntPt:     The array of active joints.
 * \param act_jnt_dof:   The array of active degree of freedom in the joints
 * \param Dofactiv:      The array of active degree of freedom in a robot.
 * \param nb_Dval:       The number of float parameters.
 * \param Dval:          The array of float parameters.
 * \param nb_Ival:       The number of integer parameters.
 * \param Ival:          The array of integer parameters.
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_create_constraint(p3d_cntrt_management * cntrt_manager,
                          const char *namecntrt, int nb_pas, p3d_jnt **pas_jntPt,
                          int *pas_jnt_dof, int *Dofpassiv, int nb_act, p3d_jnt **act_jntPt,
                          int *act_jnt_dof, int *Dofactiv, int nb_Dval, double *Dval,
                          int nb_Ival, int *Ival, int ct_num, int state) {
	if (cntrt_manager==NULL) {
		PrintWarning(("ERROR: s_p3d_create_constraint: input cntrt_manager is NULL\n"));
		return (FALSE);
	}
  
	if ((ct_num >= 0) && (ct_num > cntrt_manager->ncntrts)) {
		PrintWarning(("ERROR: s_p3d_create_constraint: wrong constraint num\n"));
		return (FALSE);
	}
	if (strcmp(namecntrt, CNTRT_FIXED_NAME) == 0) {
		return p3d_set_fixed_dof(cntrt_manager, pas_jntPt, pas_jnt_dof,
                             Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, nb_Dval, Dval,
                             ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_LIN_REL_NAME) == 0) {
		return p3d_set_lin_rel_dofs(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                Dofpassiv, nb_act, act_jntPt, act_jnt_dof, Dofactiv, nb_Dval,
                                Dval, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_REL_NAME) == 0) {
		return p3d_set_rel_dofs(cntrt_manager, pas_jntPt, pas_jnt_dof,
                            Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval[0], Dval[1],
                            Dval[2], Dval[3], ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_ON_GROUND_NAME) == 0) {
		return p3d_set_jnt_on_ground(cntrt_manager, nb_pas, pas_jntPt,
                                 pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
                                 nb_Dval, Dval, Ival, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_RRPR_NAME) == 0) {
		return p3d_set_RRPRlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
                           Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_4R_NAME) == 0) {
		return p3d_set_4Rlnk(cntrt_manager, pas_jntPt, pas_jnt_dof, Dofpassiv,
                         act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_P3R_NAME) == 0) {
		return p3d_set_P3Rlnk(cntrt_manager, pas_jntPt, pas_jnt_dof, Dofpassiv,
                          act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_3RPR_NAME) == 0) {
		return p3d_set_3RPRlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
                           Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_CAR_FRONT_WHEELS_NAME) == 0) {
		return p3d_set_car_front_wheels(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                    Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval[0], Dval[1],
                                    ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_CYCAB_WHEELS_NAME) == 0) {
		return p3d_set_cycab_wheels(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval[0], Dval[1],
                                Dval[2], ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_PLANAR_CLOSED_CHAIN_NAME) == 0) {
		return p3d_set_planar_closed_chain(cntrt_manager, pas_jntPt,
                                       pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
                                       Ival[0], ct_num, state);
	}
	/* -- functions for RLG test -- */
	if (strcmp(namecntrt, CNTRT_IN_SPHERE_NAME) == 0) {
		return p3d_set_in_sphere(cntrt_manager, pas_jntPt, pas_jnt_dof,
                             Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval[0], Dval[1],
                             Dval[2], ct_num, state);
	}
	/* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */
	if (strcmp(namecntrt, CNTRT_JNTS_RELPOS_BOUND) == 0) {
		return p3d_set_jnts_relpos_bound(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval, Ival,
                                     ct_num, state);
	}
	/* -- functions for cntrt fixing the distance range of 2 jnts relative position -- */
	if (strcmp(namecntrt, CNTRT_FIX_JNTS_RELPOS) == 0) {
		return p3d_set_fix_jnts_relpos(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                   Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval, Ival,
                                   ct_num, state);
	}
	/* -- functions for 3R IK -- */
	if (strcmp(namecntrt, CNTRT_3R_ARM_NAME) == 0) {
		return p3d_set_3R_arm_ik(cntrt_manager, nb_pas, pas_jntPt, pas_jnt_dof,
                             Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval, Ival,
                             ct_num, state);
	}
	/* -- functions for R6 IK -- */
	if (strcmp(namecntrt, CNTRT_R6_ARM_NAME) == 0) {
		return p3d_set_R6_arm_ik(cntrt_manager, nb_pas, pas_jntPt, pas_jnt_dof,
                             Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval, Ival,
                             ct_num, state);
	}
	/* -- functions for prismatic actuator IK -- */
	if (strcmp(namecntrt, CNTRT_PRISMATIC_ACTUATOR_NAME) == 0) {
		return p3d_set_prismatic_actuator_ik(cntrt_manager, pas_jntPt,
                                         pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval,
                                         ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_PRISMATIC_ACTUATOR_II_NAME) == 0) {
		return p3d_set_prismatic_actuator_II_ik(cntrt_manager, pas_jntPt,
                                            pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
                                            Dval[0], ct_num, state);
	}
	/* -- functions for 6R_bio_ik -- */
	if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME) == 0) {
		return p3d_set_6R_bio_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                             Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME_NOPEP) == 0) {
		return p3d_set_6R_bio_ik_nopep(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                   Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_6R_BIO_IK_NAME_NOPEP_NEW) == 0) {
		return p3d_set_6R_bio_ik_nopep_new(cntrt_manager, pas_jntPt,
                                       pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval,
                                       ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_BIO_BKB_HBOND) == 0) {
		return p3d_set_bio_bkb_Hbond_cntrt(cntrt_manager, pas_jntPt,
                                       pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval,
                                       Ival, ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_BIO_DIS_BOND) == 0) {
		return p3d_set_bio_diS_bond_cntrt(cntrt_manager, pas_jntPt,
                                      pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval,
                                      Ival, ct_num, state);
	}
	/* -- functions for CTBot IK -- */
	if (strcmp(namecntrt, CNTRT_CTBOT_NAME) == 0) {
		return p3d_set_CTBot_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                            Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval, ct_num,
                            state);
	}
	/*modif mokhtar*/
	if (strcmp(namecntrt, CNTRT_MIN_MAX_NAME) == 0) {
		return p3d_set_min_max_dofs(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval[0], Dval[1],
                                ct_num, state);
	}
	if (strcmp(namecntrt, CNTRT_KUKA_ARM_IK_NAME) == 0) {
		return p3d_set_kuka_arm_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                               Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Ival, Dval,
                               ct_num, state);
	}
#if defined(USE_GBM)
	/* -- functions for LWR ARM IK -- */
	if (strcmp(namecntrt, CNTRT_LWR_ARM_IK_NAME) == 0) {
		return p3d_set_lwr_arm_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                              Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Ival, Dval,
                              ct_num, state);
	}
  /* -- functions for PR2 ARM IK -- */
	if (strcmp(namecntrt, CNTRT_PR2_ARM_IK_NAME) == 0) {
		return p3d_set_pr2_arm_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                              Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Ival, Dval,
                              ct_num, state);
	}
#endif
	/* -- functions for HUMAN ARM IK -- */
	if (strcmp(namecntrt, CNTRT_R7_HUMAN_ARM_NAME) == 0) {
		return p3d_set_R7_human_arm_ik(cntrt_manager, nb_pas, pas_jntPt,
                                   pas_jnt_dof, Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Dval,
                                   Ival, ct_num, state);
	}
#if defined(USE_GBM)
	/* -- functions for PA10-6 ARM IK -- */
	if (strcmp(namecntrt, CNTRT_PA10_6_ARM_IK_NAME) == 0) {
		return p3d_set_pa10_6_arm_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                 Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Ival, Dval,
                                 ct_num, state);
	}
#endif
	if (strcmp(namecntrt, CNTRT_HEAD_OBJECT_TRACK_NAME) == 0) {
		return p3d_set_head_object_track(cntrt_manager, pas_jntPt, pas_jnt_dof,
                                     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv, Ival, Dval,
                                     ct_num, state);
	}
  
	/*fmodif mokhtar*/
	/* ---------------------- */
	PrintWarning(("ERROR: p3d_create_constraint: wrong constraint name\n"));
	return (FALSE);
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to update a constraint.
 *
 * \param ct:    The constraint.
 * \param state: The state (1 active).
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_update_constraint(p3d_cntrt * ct, int state) {
	return p3d_create_constraint(ct->cntrt_manager, ct->namecntrt,
                               ct->npasjnts, ct->pasjnts, ct->pas_jnt_dof, ct->pas_rob_dof,
                               ct->nactjnts, ct->actjnts, ct->act_jnt_dof, ct->act_rob_dof,
                               ct->ndval, ct->argu_d, ct->nival, ct->argu_i, ct->num, state);
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to introduce a kinematic constraints into the
 *         current robot constraints manager.
 *
 * \param namecntrt:     The name of the constraints.
 * \param nb_pas:        The number of passive degrees of freedom
 *                       (-1 the default number of passive degrees of freedom)
 * \param pas_jntPt:     The array of passive joints number.
 * \param pas_jnt_dof:   The array of passive degree of freedom in the joints
 * \param nb_act:        The number of active degrees of freedom
 *                       (-1 the default number of active degrees of freedom)
 * \param act_jntPt:     The array of active joints number.
 * \param act_jnt_dof:   The array of active degree of freedom in the joints
 * \param nb_Dval:       The number of float parameters.
 *                       (-1 for the default value)
 * \param Dval:          The array of float parameters.
 * \param nb_Ival:       The number of integer parameters.
 *                       (-1 for the default value)
 * \param Ival:          The array of integer parameters.
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_constraint_dof(const char *namecntrt, int nb_pas, int *pas_jnt_num,
                       int *pas_jnt_dof, int nb_act, int *act_jnt_num, int *act_jnt_dof,
                       int nb_Dval, double *Dval, int nb_Ival, int *Ival, int ct_num, int state) {
	p3d_rob *rPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	return p3d_constraint_dof_r(rPt, namecntrt, nb_pas, pas_jnt_num,
                              pas_jnt_dof, nb_act, act_jnt_num, act_jnt_dof, nb_Dval, Dval,
                              nb_Ival, Ival, ct_num, state);
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to introduce a kinematic constraints into the
 *         given robot constraints manager.
 *
 * \param robotPt:       The robot.
 * \param namecntrt:     The name of the constraints.
 * \param nb_pas:        The number of passive degrees of freedom
 *                       (-1 the default number of passive degrees of freedom)
 * \param pas_jntPt:     The array of passive joints number.
 * \param pas_jnt_dof:   The array of passive degree of freedom in the joints
 * \param nb_act:        The number of active degrees of freedom
 *                       (-1 the default number of active degrees of freedom)
 * \param act_jntPt:     The array of active joints number.
 * \param act_jnt_dof:   The array of active degree of freedom in the joints
 * \param nb_Dval:       The number of float parameters.
 *                       (-1 for the default value)
 * \param Dval:          The array of float parameters.
 * \param nb_Ival:       The number of integer parameters.
 *                       (-1 for the default value)
 * \param Ival:          The array of integer parameters.
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_constraint_dof_r(p3d_rob *robotPt, const char *namecntrt, int nb_pas,
                         int *pas_jnt_num, int *pas_jnt_dof, int nb_act, int *act_jnt_num,
                         int *act_jnt_dof, int nb_dval, double *Dval, int nb_ival, int *Ival,
                         int ct_num, int state) {
	p3d_jnt * pas_jntPt[MAX_ARGU_CNTRT];
	p3d_jnt * act_jntPt[MAX_ARGU_CNTRT];
	int Dofpassiv[MAX_ARGU_CNTRT];
	int Dofactiv[MAX_ARGU_CNTRT];
	int nb_passif, nb_actif, nb_Dval, nb_Ival, i;
	int valid;
  
	p3d_constraint_get_nb_param(namecntrt, &nb_passif, &nb_actif, &nb_Dval,
                              &nb_Ival);
	/* Check the number of parameters */
	if ((nb_passif != nb_pas) && (nb_pas != -1)) {
		valid = FALSE;
		if ((strcmp(namecntrt, CNTRT_R6_ARM_NAME) == 0) && (nb_pas == 6)) {
			valid = TRUE;
		} else if ((strcmp(namecntrt, CNTRT_ON_GROUND_NAME) == 0) && (nb_pas
                                                                  >= 0)) {
			valid = TRUE;
		} else if ((strcmp(namecntrt, CNTRT_R7_HUMAN_ARM_NAME) == 0) && (nb_pas
                                                                     == 7)) {
			valid = TRUE;
		} else if (!valid) {
			PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
                    "passif degree of freedom !!!\n"));
			printf("%s: %d \n",__FILE__ ,__LINE__);
			return FALSE;
		}
		nb_passif = nb_pas;
	}
	if ((nb_actif != nb_act) && (nb_act != -1)) {
		valid = FALSE;
		if ((strcmp(namecntrt, CNTRT_LIN_REL_NAME) == 0) && (nb_act >= 1)) {
			valid = TRUE;
		}
		if (!valid) {
			PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
                    "actif degree of freedom !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		nb_actif = nb_act;
	}
	if ((nb_Dval != nb_dval) && (nb_dval != -1)) {
		valid = FALSE;
		if ((strcmp(namecntrt, CNTRT_LIN_REL_NAME) == 0) && (nb_dval == nb_actif + 1)) {
			valid = TRUE;
		} else if ((strcmp(namecntrt, CNTRT_FIXED_NAME) == 0) && robotPt->joints[pas_jnt_num[0]]->dof_equiv_nbr == nb_dval) {
			//there is only one joint in the fixed constraint
			valid = TRUE;
		}
		if (!valid) {
			PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
                    "float parameters !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		nb_Dval = nb_dval;
	}
	if ((nb_Ival != nb_ival) && (nb_ival != -1)) {
		valid = FALSE;
		if ((strcmp(namecntrt, CNTRT_R6_ARM_NAME) == 0) && nb_ival == 3) {
			//The kinematic class is defined using 3 parameters
			valid = TRUE;
		}
		if (!valid) {
			PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
                    "integer parameters !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
	}
  
	for (i = 0; i < nb_passif; i++) {
		if ((pas_jnt_num[i] < 0) || (pas_jnt_num[i] > robotPt->njoints)) {
			PrintWarning(("ERROR: p3d_constraint_dof: joint not valid !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		pas_jntPt[i] = robotPt->joints[pas_jnt_num[i]];
		if ((pas_jnt_dof[i] < 0) || (pas_jnt_dof[i] > pas_jntPt[i]->dof_equiv_nbr)) {
			PrintWarning(("ERROR: p3d_constraint_dof: dof not valid !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		Dofpassiv[i] = pas_jntPt[i]->index_dof + pas_jnt_dof[i];
	}
	for (i = 0; i < nb_actif; i++) {
		if ((act_jnt_num[i] < 0) || (act_jnt_num[i] > robotPt->njoints)) {
			PrintWarning(("ERROR: p3d_constraint_dof: joint not valid !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		act_jntPt[i] = robotPt->joints[act_jnt_num[i]];
		if ((act_jnt_dof[i] < 0) || (act_jnt_dof[i] > act_jntPt[i]->dof_equiv_nbr)) {
			PrintWarning(("ERROR: p3d_constraint_dof: dof not valid !!!\n")); printf("%s: %d \n",__FILE__,__LINE__);
			return FALSE;
		}
		Dofactiv[i] = act_jntPt[i]->index_dof + act_jnt_dof[i];
	}
  
	//  for(int i=0;i<nb_Dval;i++)
	//  {
	//      std::cout << "Dofpassiv["<<i<<"] = "<< pas_jnt_dof[i] << std::endl;
	//  }
	return p3d_create_constraint(robotPt->cntrt_manager, namecntrt,
                               nb_passif, pas_jntPt, pas_jnt_dof, Dofpassiv,
                               nb_actif, act_jntPt, act_jnt_dof, Dofactiv,
                               nb_Dval, Dval, nb_Ival, Ival, ct_num, state);
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to introduce a kinematic constraints into the
 *         current robot constraints manager.
 * 
 * \note Use the first degree of freedom of each joint
 *
 * \param namecntrt:     The name of the constraints.
 * \param nb_pas:        The number of passive degrees of freedom
 *                       (-1 the default number of passive degrees of freedom)
 * \param Jpasiv:        The array of passive joints number.
 * \param nb_act:        The number of active degrees of freedom
 *                       (-1 the default number of active degrees of freedom)
 * \param Jactiv:        The array of active joints number.
 * \param nb_Dval:       The number of float parameters.
 *                       (-1 for the default value)
 * \param Dval:          The array of float parameters.
 * \param nb_Ival:       The number of integer parameters.
 *                       (-1 for the default value)
 * \param Ival:          The array of integer parameters.
 *
 * \return TRUE if success, FALSE if it fails.
 */

int p3d_constraint(const char *namecntrt, int nb_pas, int *Jpasiv, int nb_act,
                   int *Jactiv, int nb_dval, double *Dval, int nb_ival, int *Ival,
                   int ct_num, int state) {
	p3d_rob *rPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	return p3d_constraint_r(rPt, namecntrt, nb_pas, Jpasiv, nb_act, Jactiv,
                          nb_dval, Dval, nb_ival, Ival, ct_num, state);
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to introduce a kinematic constraints into the
 *         given robot constraints manager.
 *
 * \note Use the first degree of freedom of each joint
 *
 * \param robotPt:       The robot.
 * \param namecntrt:     The name of the constraints.
 * \param nb_pas:        The number of passive degrees of freedom
 *                       (-1 the default number of passive degrees of freedom)
 * \param Jpasiv:        The array of passive joints number.
 * \param nb_act:        The number of active degrees of freedom
 *                       (-1 the default number of active degrees of freedom)
 * \param Jactiv:        The array of active joints number.
 * \param nb_Dval:       The number of float parameters.
 *                       (-1 for the default value)
 * \param Dval:          The array of float parameters.
 * \param nb_Ival:       The number of integer parameters.
 *                       (-1 for the default value)
 * \param Ival:          The array of integer parameters.
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_constraint_r(p3d_rob *robotPt, const char *namecntrt, int nb_pas,
                     int *Jpasiv, int nb_act, int *Jactiv, int nb_dval, double *Dval,
                     int nb_ival, int *Ival, int ct_num, int state) {
	int jnt_dof_passif[MAX_ARGU_CNTRT];
	int jnt_dof_actif[MAX_ARGU_CNTRT];
	int i;
  
	for (i = 0; i < MAX_ARGU_CNTRT; i++) {
		jnt_dof_passif[i] = 0;
		jnt_dof_actif[i] = 0;
	}
  
	return p3d_constraint_dof_r(robotPt, namecntrt, nb_pas, Jpasiv,
                              jnt_dof_passif, nb_act, Jactiv, jnt_dof_actif, nb_dval, Dval,
                              nb_ival, Ival, ct_num, state);
}

/*--------------------------------------------------------------------------*/
/*! \brief Create a constraints manager structure.
 *
 *  \param nb_dof: The number of degree of freedom in the robot
 *                 (or multi-robot)
 *
 *  \return The new constraints manager (NULL if there is a memory erreor).
 */
p3d_cntrt_management * p3d_create_cntrt_manager(int nb_dof) {
	p3d_cntrt_management * cntrt_manager;
	int i;
  
	cntrt_manager = MY_ALLOC(p3d_cntrt_management, 1);
	if (cntrt_manager == NULL) {
		PrintError(("Not enough memory !!!\n"));
		return NULL;
	}
	cntrt_manager->in_cntrt = MY_ALLOC(int, nb_dof);
	if (cntrt_manager->in_cntrt == NULL) {
		PrintError(("Not enough memory !!!\n"));
		return NULL;
	}
	cntrt_manager->cntrts = NULL;
	cntrt_manager->ncntrts = 0;
	cntrt_manager->nb_dof = nb_dof;
	for (i = 0; i < nb_dof; i++) {
		cntrt_manager->in_cntrt[i] = DOF_WITHOUT_CNTRT;
	}
  
	cntrt_manager->cntrt_call_list = dbl_list_pointer_init();
  
	return (cntrt_manager);
}

/*--------------------------------------------------------------------------*/
/*! \brief Destroy a constraints manager structure.
 *
 *  \param cntrt_manager: The constraints manager.
 */
void p3d_destroy_cntrt_manager(p3d_cntrt_management * cntrt_manager) {
  
	if (p3d_clear_cntrt_manager(cntrt_manager)) {
		MY_FREE(cntrt_manager->in_cntrt, int, cntrt_manager->nb_dof);
		cntrt_manager->in_cntrt = NULL;
		MY_FREE(cntrt_manager, p3d_cntrt_management, 1);
	}
}

/*--------------------------------------------------------------------------*/
/*! \brief Clear a constraints manager structure.
 *
 *  Destroy all the constraints.
 *
 *  \param cntrt_manager: The constraints manager.
 *  \return TRUE if the cntrt manager is cleared, FALSE otherwise
 */
int p3d_clear_cntrt_manager(p3d_cntrt_management * cntrt_manager) {
	int i = 0;
	pp3d_cntrt ct;
  
	if (cntrt_manager != NULL) {
		for (i = (cntrt_manager->ncntrts) - 1; i >= 0; i--) {
			ct = cntrt_manager->cntrts[i];
			if (ct->enchained != NULL) {
				MY_FREE(ct->enchained, p3d_cntrt *, ct->nenchained);
			}
			if (ct->rlgPt != NULL) {
				p3d_destroy_rlg(ct->rlgPt);
			}
			if (ct->parallel_sys_data != NULL) {
				p3d_destroy_parallel_sys_data(ct->parallel_sys_data);
			}
			if (ct->bio_ik_data != NULL) {
				MY_FREE(ct->bio_ik_data, bio_6R_ik_data, 1);
			}
			MY_FREE(ct, p3d_cntrt, 1);
		}
    
		dbl_list_clear(cntrt_manager->cntrt_call_list);
		//     cntrt_manager->cntrt_call_list = dbl_list_pointer_init();
		MY_FREE(cntrt_manager->cntrts, p3d_cntrt *, cntrt_manager->ncntrts);
		cntrt_manager->ncntrts = 0;
		cntrt_manager->cntrts = NULL;
		for (i = 0; i < cntrt_manager->nb_dof; i++) {
			cntrt_manager->in_cntrt[i] = DOF_WITHOUT_CNTRT;
		}
		return TRUE;
	}
	return FALSE;
}

/*--------------------------------------------------------------------------*/
/*! \brief Copy a constraints of a constraints manager into an other.
 *
 *  \note Destroy all the constraints of \a cntrt_manager_destPt.
 *
 *  \param cntrt_manager_srcPt: The constraints manager to be copied.
 *  \param cntrt_manager_destPt: The constraints manager source.
 *
 *  \return TRUE if all the constraints have been copied successfully.
 */
int p3d_copy_cntrt_manager_into(p3d_cntrt_management * cntrt_manager_srcPt,
                                p3d_cntrt_management * cntrt_manager_destPt) {
	int j, test;
	p3d_cntrt * cntrtPt;
  
	if ((cntrt_manager_srcPt == NULL) || (cntrt_manager_destPt == NULL)
			|| (cntrt_manager_srcPt->nb_dof != cntrt_manager_destPt->nb_dof)) {
		return FALSE;
	}
	p3d_clear_cntrt_manager(cntrt_manager_destPt);
  
	/* copy the constraints */
	test = TRUE;
	for (j = 0; j < cntrt_manager_srcPt->ncntrts; j++) {
		cntrtPt = cntrt_manager_srcPt->cntrts[j];
		if (cntrtPt != NULL) {
			if (!p3d_create_constraint(cntrt_manager_destPt,
                                 cntrtPt->namecntrt, cntrtPt->npasjnts, cntrtPt->pasjnts,
                                 cntrtPt->pas_jnt_dof, cntrtPt->pas_rob_dof,
                                 cntrtPt->nactjnts, cntrtPt->actjnts, cntrtPt->act_jnt_dof,
                                 cntrtPt->act_rob_dof, cntrtPt->ndval, cntrtPt->argu_d,
                                 cntrtPt->nival, cntrtPt->argu_i, -1, cntrtPt->active)) {
				test = FALSE;
			}
		}
	}
	return test;
}

/* ------------------------------------------------------------------ */

p3d_cntrt *p3d_get_current_cntrt() {
	return last_cntrt_set;
}

int p3d_set_current_cntrt(p3d_cntrt *ct) {
	last_cntrt_set = ct;
	return TRUE;
}

/* ------------------------------------------------------------------ */

/* function that fills the field Tatt of a cntrt  */
/* Tatt is the transformation between a joint and */
/* the frame asociated to the grasp               */
int p3d_set_cntrt_Tatt(int ct_num, double *matelem) {
	p3d_rob *r;
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	return p3d_set_cntrt_Tatt_r(r, ct_num, matelem);
}

int p3d_set_cntrt_Tatt_r(p3d_rob *r, int ct_num, double *matelem) {
	p3d_cntrt *ct;
  
	if ((r->cntrt_manager->cntrts == NULL) || (r->cntrt_manager->cntrts[ct_num]
                                             == NULL))
		return (FALSE);
  
	ct = r->cntrt_manager->cntrts[ct_num];
  
	ct->Tatt[0][0] = matelem[0];
	ct->Tatt[0][1] = matelem[1];
	ct->Tatt[0][2] = matelem[2];
	ct->Tatt[0][3] = matelem[3];
	ct->Tatt[1][0] = matelem[4];
	ct->Tatt[1][1] = matelem[5];
	ct->Tatt[1][2] = matelem[6];
	ct->Tatt[1][3] = matelem[7];
	ct->Tatt[2][0] = matelem[8];
	ct->Tatt[2][1] = matelem[9];
	ct->Tatt[2][2] = matelem[10];
	ct->Tatt[2][3] = matelem[11];
	ct->Tatt[3][0] = 0;
	ct->Tatt[3][1] = 0;
	ct->Tatt[3][2] = 0;
	ct->Tatt[3][3] = 1;
	
        p3d_mat4Copy(ct->Tatt,ct->Tatt_default);
	
	p3d_set_cntrt_Tsing(ct);
  
	return (TRUE);
}

/* function that fills the field Tatt2 of a cntrt   */
/* Tatt2 is the transformation between a joint and  */
/* the frame asociated to the base                  */
int p3d_set_cntrt_Tatt2(int ct_num, double *matelem) {
	p3d_rob *r;
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	return p3d_set_cntrt_Tatt2_r(r, ct_num, matelem);
}

int p3d_set_cntrt_Tatt2_r(p3d_rob *r, int ct_num, double *matelem) {
	p3d_cntrt *ct;
  
	if ((r->cntrt_manager->cntrts == NULL) || (r->cntrt_manager->cntrts[ct_num]
                                             == NULL))
		return (FALSE);
  
	ct = r->cntrt_manager->cntrts[ct_num];
  
	ct->Tatt2[0][0] = matelem[0];
	ct->Tatt2[0][1] = matelem[1];
	ct->Tatt2[0][2] = matelem[2];
	ct->Tatt2[0][3] = matelem[3];
	ct->Tatt2[1][0] = matelem[4];
	ct->Tatt2[1][1] = matelem[5];
	ct->Tatt2[1][2] = matelem[6];
	ct->Tatt2[1][3] = matelem[7];
	ct->Tatt2[2][0] = matelem[8];
	ct->Tatt2[2][1] = matelem[9];
	ct->Tatt2[2][2] = matelem[10];
	ct->Tatt2[2][3] = matelem[11];
	ct->Tatt2[3][0] = 0;
	ct->Tatt2[3][1] = 0;
	ct->Tatt2[3][2] = 0;
	ct->Tatt2[3][3] = 1;
  
	return (TRUE);
}

/****************************************************************************/
/** \brief Fill the Tsingularity Matrix.
 The matrix TSingularity is filled at constraint creation by an offset matrix
 or by the identity matrix
 \param *ct the constraint
 */
/****************************************************************************/
static void p3d_set_cntrt_Tsing(p3d_cntrt *ct) {
	p3d_matrix4 tmp1;
  
	p3d_mat4Copy(ct->TSingularity, tmp1);
	p3d_matInvertXform(ct->Tatt, ct->TSingularity);
	ct->TSingularity[0][3] += tmp1[0][3];
	ct->TSingularity[1][3] += tmp1[1][3];
	ct->TSingularity[2][3] += tmp1[2][3];
	return;
}

/**
 * @brief Add a singularity to a constraint
 * @param cntrt the constraint number
 * @param nJnt the number of jnts for this singularity
 * @param jnts the jnts numbers
 * @param values the jnt dof values
 */
void p3d_set_singularity(int constraint, int nJnt, int *jnts, double *values) {
	int i = 0, j = 0, nDof = 0, n = 0;
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_cntrt *cntrt = r->cntrt_manager->cntrts[constraint];
	p3d_singularity *singularity = NULL;
  
	cntrt->singularities[cntrt->nSingularities] = MY_ALLOC(p3d_singularity, 1);
	singularity = cntrt->singularities[cntrt->nSingularities];
	singularity->nJnt = nJnt;
	for (i = 0; i < nJnt; i++, n += nDof) {
		singularity->singJntVal[i] = MY_ALLOC(p3d_singJntVal, 1);
		(singularity->singJntVal[i])->jntNum = jnts[i];
		nDof = (r->joints[jnts[i]])->dof_equiv_nbr;
		(singularity->singJntVal[i])->val = MY_ALLOC(double, nDof);
		for (j = 0; j < nDof; j++) {
			(singularity->singJntVal[i])->val[j] = values[n+j];
		}
	}
	cntrt->nSingularities++;
	singularity->nRel = 0;
}

void p3d_set_singular_rel(int constraint, int singNum, int nPaires,
                          int *classes) {
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_cntrt *cntrt = r->cntrt_manager->cntrts[constraint];
	p3d_singularity *singularity = cntrt->singularities[singNum];
  
	singularity->nRel = nPaires;
	for (int i = 0; i < nPaires; i++) {
		singularity->classes[i][0] = classes[i*2];
		singularity->classes[i][1] = classes[i*2+1];
	}
}
/* ------------------------------------------------------------------ */

int p3d_update_all_jnts_state(int mode) {
	p3d_rob *r;
	int i;
  
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	for (i = 0; i < r->cntrt_manager->ncntrts; i++) {
		/* mode == 0 -> normal case      -> update all cntrts */
		/* mode == 1 -> from rw_scenario -> not need to update active cntrts */
		if ((mode == 0) || !(r->cntrt_manager->cntrts[i]->active)) {
			p3d_update_jnts_state(r->cntrt_manager,
                            r->cntrt_manager->cntrts[i],
                            r->cntrt_manager->cntrts[i]->active);
		}
	}
  
	return TRUE;
}

int p3d_update_jnts_state(p3d_cntrt_management * cntrt_manager, p3d_cntrt *ct,
                          int cntrt_state) {
	int i, j;
	int before[MAX_ARGU_CNTRT];
  
	if (cntrt_state == 0) {
		for (i = 0; i < ct->npasjnts; i++) {
			int nbCntrts = 0, desactive = 1;
			p3d_cntrt** cntrts = p3d_getJointCntrts(cntrt_manager,
                                              ct->pasjnts[i]->num, &nbCntrts);
			//      std::cout << "number of constraints : " << nbCntrts << std:: endl;
			for (j = 0; j < nbCntrts; j++) {
				if (cntrts[j]->num != ct->num && cntrts[j]->active == 1) {
					desactive = 0;
					break;
				}
			}
			MY_FREE(cntrts, p3d_cntrt*, nbCntrts);
			if (desactive) {
				cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] = DOF_WITHOUT_CNTRT;
				//        std::cout << "Put active : " << ct->pas_rob_dof[i] << std::endl;
				//        std::cout << ct->namecntrt << std::endl;
				//        std::cout << ct->ndval << std::endl;
				if (strcmp(ct->namecntrt, "p3d_fixed_jnt")==0) {
					for (int k=1; k<ct->ndval; k++) {
						cntrt_manager->in_cntrt[ct->pas_rob_dof[i]+k]
            = DOF_WITHOUT_CNTRT;
					}
				}
			}
		}
		if (ct->enchained != NULL) {
			for (i = 0; i < ct->nenchained; i++) {
				for (j = 0; j < ct->enchained[i]->nactjnts; j++) {
					cntrt_manager->in_cntrt[ct->enchained[i]->act_rob_dof[j]]
          = DOF_ACTIF;
				}
			}
		}
	} else { /* cntrt_state == 1 */
		//verifier si il n'y a pas d'autres contraintes pour le meme joint actives
		for (i = 0; i < ct->npasjnts; i++) {
			if (cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] != DOF_PASSIF) {
				before[i] = cntrt_manager->in_cntrt[ct->pas_rob_dof[i]];
				cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] = DOF_PASSIF;
			} else {
				int nbCntrts = 0;
				p3d_cntrt** cntrts = p3d_getJointCntrts(cntrt_manager,
                                                ct->pasjnts[i]->num, &nbCntrts);
				for (j = 0; j < nbCntrts; j++) {
					if (cntrts[j]->num != ct->num) {
						//désactiver la contrainte
						if (p3d_update_constraint(cntrts[j], 0)) {
							if (cntrts[j]->enchained != NULL) {
								p3d_unchain_cntrts(cntrts[j]);
							}
							cntrt_manager->in_cntrt[ct->pas_rob_dof[i]]
              = DOF_PASSIF;
							if (strcmp(ct->namecntrt, "p3d_fixed_jnt")==0) {
								for (int k=1; k<ct->ndval; k++) {
									cntrt_manager->in_cntrt[ct->pas_rob_dof[i]+k]
                  = DOF_PASSIF;
								}
							}
						}
					}
				}
				MY_FREE(cntrts, p3d_cntrt*, nbCntrts);
				//         for (j = 0;j < i;j++) {
				//           ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[j]] = before[j];
				//         }
				//         PrintInfo(("ERROR : Joint %d is already passive\n", ct->pasjnts[i]->num));
				//         return(FALSE);
			}
		}
	}
	return (TRUE);
}

/****************************************************************************/
/** \brief Change the state of the active DoF given as parameter
 \param *ct the constraint
 \param rob_dof the degree of freedom to change
 \param state the new state
 \return true if the state is changed successfully false otherwise
 */
/****************************************************************************/
static int p3d_change_act_rob_dof_state(p3d_cntrt *ct, int rob_dof, int state) {
	int i;
	//faut il peut etre changer l'etat du Dof aussi dans le cntrt_manager ?
	for (i = 0; i < ct->nactjnts; i++) {
		if (ct->act_rob_dof[i] == rob_dof) {
			ct->actjnt_state[i] = state;
			return (TRUE);
		}
	}
	return (FALSE);
}

/****************************************************************************/
/** \brief Search if the constraints depend on the modified joint
 \param *ct the constraint
 \return true if we have to execute the constraint function false otherwise
 */
/****************************************************************************/
static int p3d_go_into_cntrt_fct(p3d_cntrt *ct) {
	int i, modif;
  
	modif = 0;
	for (i = 0; i < ct->nactjnts; i++) {
		if (p3d_jnt_get_dof_is_modified(ct->actjnts[i], ct->act_jnt_dof[i])) {
			if (ct->actjnt_state[i] == 0)
				return FALSE;
			modif = 1;
		}
	}
	return (modif);
}

/* ------------------------------------------------------------------ */

void p3d_col_deactivate_cntrt_manager_pairs(p3d_cntrt_management *
                                            cntrt_manager) {
	p3d_cntrt *ct;
  
	if ((cntrt_manager != NULL) && (cntrt_manager->cntrts != NULL)) {
		ct = cntrt_manager->cntrts[0];
		while (ct != NULL) {
			if (ct->active) {
				p3d_col_deactivate_one_cntrt_pairs(ct);
			}
			ct = ct->next_cntrt;
		}
	}
}

/*--------------------------------------------------------------------------*/
/*! \brief Deactive the collisions pair in the given context
 *         for all the constraints in the constraints manager
 *
 *  \param cntrt_manager: the constraints manager
 *  \param col_pairPt:    the collision context
 */
/*--------------------------------------------------------------------------*/
void p3d_col_deactivate_cntrt_manager_pairs_into(
                                                 p3d_cntrt_management * cntrt_manager, p3d_collision_pair * col_pairPt) {
	p3d_cntrt *ct;
  
	if ((cntrt_manager != NULL) && (cntrt_manager->cntrts != NULL)) {
		ct = cntrt_manager->cntrts[0];
		while (ct != NULL) {
			if (ct->active) {
				p3d_col_deactivate_one_cntrt_pairs_into(ct, col_pairPt);
			}
			ct = ct->next_cntrt;
		}
	}
}

void p3d_col_deactivate_cntrt_pairs(void) {
	p3d_rob *r;
  
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_col_deactivate_cntrt_manager_pairs(r->cntrt_manager);
}

/*--------------------------------------------------------------------------*/
/*! \brief Deactive the collisions pair in the given context
 *         for one constraint
 *
 *  \param ct:         the constraints
 *  \param col_pairPt: the collision context
 */
void p3d_col_deactivate_one_cntrt_pairs_into(p3d_cntrt *ct,
                                             p3d_collision_pair * col_pairPt) {
	int i;
  
	for (i = 0; (i < MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
		p3d_col_pair_deactivate_pair(col_pairPt, ct->col_pairs[0][i],
                                 ct->col_pairs[1][i]);
	}
	if (ct->enchained != NULL) {
		if (strcmp(ct->namecntrt, "p3d_planar_closed_chain") != 0) {
			for (i = 0; i < ct->nenchained; i++) {
				p3d_col_deactivate_enchained_cntrts_pairs(ct, ct->enchained[i],
                                                  ct->enchained_rob_dof[i]);
			}
		}
	}
}

void p3d_col_deactivate_one_cntrt_pairs(p3d_cntrt *ct) {
	int i;
  
	for (i = 0; (i < MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
		p3d_col_deactivate_obj_obj(ct->col_pairs[0][i], ct->col_pairs[1][i]);
	}
	if (ct->enchained != NULL) {
		if (strcmp(ct->namecntrt, "p3d_planar_closed_chain") != 0) {
			for (i = 0; i < ct->nenchained; i++) {
				p3d_col_deactivate_enchained_cntrts_pairs(ct, ct->enchained[i],
                                                  ct->enchained_rob_dof[i]);
			}
		}
	}
}

void p3d_col_activate_one_cntrt_pairs(p3d_cntrt *ct) {
	int i;
  
	for (i = 0; (i < MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
		p3d_col_activate_obj_obj(ct->col_pairs[0][i], ct->col_pairs[1][i]);
	}
	if (ct->enchained != NULL) {
		if (strcmp(ct->namecntrt, "p3d_planar_closed_chain") != 0) {
			for (i = 0; i < ct->nenchained; i++) {
				p3d_col_activate_enchained_cntrts_pairs(ct, ct->enchained[i],
                                                ct->enchained_rob_dof[i]);
			}
		}
	}
}

/*--------------------------------------------------------------------------*/
/*! \brief Deactive the collisions pair in the given context
 *         for chained constraints
 *
 *  \param ct:         the constraints
 *  \param ect:        the constraints linked to \a ct
 *  \param rob_dof:    the configuration indice linked
 *  \param col_pairPt: the collision context
 */
void p3d_col_deactivate_enchained_cntrts_pairs_into(p3d_cntrt *ct,
                                                    p3d_cntrt *ect, int rob_dof, p3d_collision_pair * col_pairPt) {
	int i, j;
  
	if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
		for (i = 0; i < ct->nactjnts; i++) {
			if (ct->act_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_pair_deactivate_pair(col_pairPt,
                                         ct->actjnts[i]->o, ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_pair_deactivate_pair(col_pairPt,
                                         ct->actjnts[i]->o, ect->actjnts[j]->o);
					}
				}
			}
		}
		for (i = 0; i < ct->npasjnts; i++) {
			if (ct->pas_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_pair_deactivate_pair(col_pairPt,
                                         ct->pasjnts[i]->o, ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_pair_deactivate_pair(col_pairPt,
                                         ct->pasjnts[i]->o, ect->actjnts[j]->o);
					}
				}
			}
		}
	}
}

void p3d_col_deactivate_enchained_cntrts_pairs(p3d_cntrt *ct, p3d_cntrt *ect,
                                               int rob_dof) {
	int i, j;
  
	if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
		for (i = 0; i < ct->nactjnts; i++) {
			if (ct->act_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_deactivate_obj_obj(ct->actjnts[i]->o,
                                       ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_deactivate_obj_obj(ct->actjnts[i]->o,
                                       ect->actjnts[j]->o);
					}
				}
			}
		}
		for (i = 0; i < ct->npasjnts; i++) {
			if (ct->pas_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_deactivate_obj_obj(ct->pasjnts[i]->o,
                                       ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_deactivate_obj_obj(ct->pasjnts[i]->o,
                                       ect->actjnts[j]->o);
					}
				}
			}
		}
	}
}

void p3d_col_activate_enchained_cntrts_pairs(p3d_cntrt *ct, p3d_cntrt *ect,
                                             int rob_dof) {
	int i, j;
  
	if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
		for (i = 0; i < ct->nactjnts; i++) {
			if (ct->act_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_activate_obj_obj(ct->actjnts[i]->o,
                                     ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_activate_obj_obj(ct->actjnts[i]->o,
                                     ect->actjnts[j]->o);
					}
				}
			}
		}
		for (i = 0; i < ct->npasjnts; i++) {
			if (ct->pas_rob_dof[i] != rob_dof) {
				for (j = 0; j < ect->npasjnts; j++) {
					if (ect->pas_rob_dof[j] != rob_dof) {
						p3d_col_activate_obj_obj(ct->pasjnts[i]->o,
                                     ect->pasjnts[j]->o);
					}
				}
				for (j = 0; j < ect->nactjnts; j++) {
					if (ect->act_rob_dof[j] != rob_dof) {
						p3d_col_activate_obj_obj(ct->pasjnts[i]->o,
                                     ect->actjnts[j]->o);
					}
				}
			}
		}
	}
}

/* ------------------------------------------------------------------ */
/*! \brief Add a constraint into another constraint chain.
 *  \param ct:         the constraints
 *  \param ect:        the constraints linked to \a ct
 *  \param rob_dof:    the configuration indice linked
 */
/* ------------------------------------------------------------------ */
static void p3d_add_to_cntrts_chain(p3d_cntrt *ct, p3d_cntrt *ect, int rob_dof) {
  for(int i = 0; i < ct->nenchained; i++){
    if(ct->enchained[i] == ect){
      return; //the constraint is already in the enchained list
    }
  }

	ct->enchained = MY_REALLOC(ct->enchained, pp3d_cntrt, ct->nenchained,
                             ct->nenchained + 1);
	ct->enchained[ct->nenchained] = ect;
	ct->enchained_rob_dof[ct->nenchained] = rob_dof;
	ct->nenchained++;
}

/* ------------------------------------------------------------------ */
/*! \brief Check if \a etc is a independent constraint or not
 *  \param ect:        the constraints
 *  \param rob_dof:    the DoF indice linked
 *  \param arg:        passive 2 or active 1 DoF
 */
/* ------------------------------------------------------------------ */
void p3d_enchain_cntrt(p3d_cntrt *ect, int rob_dof, int arg) {
	p3d_cntrt_management *cntrt_manager;
	p3d_cntrt *ct;
	int j, k;
  
	cntrt_manager = ect->cntrt_manager;
	for (k = 0; k < cntrt_manager->ncntrts; k++) {
		ct = cntrt_manager->cntrts[k];
		if (arg == 1) {
			/* on cherche la premiere cntrt pour laquelle rob_dof est active */
			for (j = 0; j < ct->nactjnts; j++) {
				if (rob_dof == ct->act_rob_dof[j]) {
					p3d_add_to_cntrts_chain(ct, ect, rob_dof);
					/* on sort ect de la liste primaire de cntrts du robot
					 (si ct est active) */
					if (ct->active) {
						p3d_change_act_rob_dof_state(ect, rob_dof, 0);
						/*    ect->prev_cntrt->next_cntrt = NULL; */
					}
					return;
				}
			}
		} else if (arg == 2) {
			/* on cherche la cntrt pour laquelle J est passive
			 (il peut avoir que UNE) */
			for (j = 0; j < ct->npasjnts; j++) {
				if (rob_dof == ct->pas_rob_dof[j]) {
					p3d_add_to_cntrts_chain(ct, ect, rob_dof);
					if (ct->active) {
						p3d_change_act_rob_dof_state(ect, rob_dof, 0);
						/*    ect->prev_cntrt->next_cntrt = NULL; */
					}
					return;
				}
			}
		}
	}
}

void p3d_unchain_cntrts(p3d_cntrt *ct) {
	int i;
  
	for (i = 0; i < ct->nenchained; i++) {
		p3d_change_act_rob_dof_state(ct->enchained[i],
                                 ct->enchained_rob_dof[i], 1);
	}
}

void p3d_reenchain_cntrts(p3d_cntrt *ct) {
	int i;
  
	for (i = 0; i < ct->nenchained; i++) {
		p3d_change_act_rob_dof_state(ct->enchained[i],
                                 ct->enchained_rob_dof[i], 0);
	}
}

void p3d_activateCntrt(p3d_rob *robot, p3d_cntrt* cntrt) {
	if (p3d_update_constraint(cntrt, 1)) {
		if (cntrt->enchained != NULL)
			p3d_reenchain_cntrts(cntrt);
		p3d_col_deactivate_one_cntrt_pairs(cntrt);
	}
}

void p3d_desactivateCntrt(p3d_rob *robot, p3d_cntrt* cntrt) {
	if (p3d_update_constraint(cntrt, 0)) {
		if (cntrt->enchained != NULL)
			p3d_unchain_cntrts(cntrt);
		p3d_update_jnts_state(robot->cntrt_manager, cntrt, 0);
		p3d_col_activate_one_cntrt_pairs(cntrt);
	}
}

void p3d_desactivateAllCntrts(p3d_rob *robot) {
	for (int i = 0; i < robot->cntrt_manager->ncntrts; i++) {
		p3d_cntrt* cntrt = robot->cntrt_manager->cntrts[i];
		if (cntrt->active) {
			p3d_desactivateCntrt(robot, cntrt);
		}
	}
}

int * p3d_getActiveCntrts(p3d_rob* robot, int * nbCntrts) {
	int * tmp = MY_ALLOC(int, robot->cntrt_manager->ncntrts), *activeCntrtIds = NULL;
	*nbCntrts = 0;
  
	for (int i = 0; i < robot->cntrt_manager->ncntrts; i++) {
		p3d_cntrt* cntrt = robot->cntrt_manager->cntrts[i];
		if (cntrt->active) {
			tmp[*nbCntrts] = cntrt->num;
			(*nbCntrts)++;
		}
	}
	if (*nbCntrts != 0) {
		activeCntrtIds = MY_ALLOC(int, *nbCntrts);
		for (int i = 0; i < *nbCntrts; i++) {
			activeCntrtIds[i] = tmp[i];
		}
	}
	MY_FREE(tmp, int, robot->cntrt_manager->ncntrts);
	return activeCntrtIds;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* fonctions pour generer les structures de donees des contraintes    */

/*--------------------------------------------------------------------------*/
/*! \brief Create a new constraint structure in a constraint manager.
 *
 * \param cntrt_manager: The constraint manager.
 *
 * \return The pointer on the new constraint (NULL if there is a memory error)
 *
 * \note This function increase the array in the constraint manager
 *       to store this new constraints.
 *
 * \internal
 */
static p3d_cntrt * s_p3d_create_cntrts(p3d_cntrt_management * cntrt_manager) {
	p3d_cntrt * ct, *cct;
	int i;
  
	ct = MY_ALLOC(p3d_cntrt, 1);
	if (ct == NULL) {
		PrintError(("Not enough memory !!!\n"));
		return NULL;
	}
	if (cntrt_manager->cntrts == NULL) {
		ct->prev_cntrt = NULL;
		cntrt_manager->cntrts = MY_ALLOC(pp3d_cntrt, 1);
	} else {
		cntrt_manager->cntrts = MY_REALLOC(cntrt_manager->cntrts, pp3d_cntrt,
                                       cntrt_manager->ncntrts, cntrt_manager->ncntrts + 1);
		cct = cntrt_manager->cntrts[0];
		while (cct->next_cntrt != NULL) {
			cct = cct->next_cntrt;
		}
		cct->next_cntrt = ct;
		ct->prev_cntrt = cct;
	}
	dbl_list_add_link(cntrt_manager->cntrt_call_list, ct);
	ct->next_cntrt = NULL;
	ct->num = cntrt_manager->ncntrts;
	cntrt_manager->ncntrts++;
	cntrt_manager->cntrts[ct->num] = ct;
  
	ct->namecntrt[0] = '\0'; /* No name */
	ct->active = FALSE;
	ct->fct_cntrt = NULL;
	ct->nactjnts = 0;
	ct->npasjnts = 0;
	ct->ndval = 0;
	ct->nival = 0;
	ct->nenchained = 0;
	ct->enchained = NULL;
	ct->reshoot_ct = NULL;
	ct->nctud = 0;
	ct->ct_to_update = NULL;
	ct->rlgPt = NULL;
	ct->parallel_sys_data = NULL;
	ct->bio_ik_data = NULL;
	p3d_mat4Copy(p3d_mat4IDENTITY, ct->Tatt);
  p3d_mat4Copy(p3d_mat4IDENTITY, ct->Tatt2);
	ct->cntrt_manager = cntrt_manager;
	for (i = 0; i < MAX_ARGU_CNTRT; i++) {
		ct->actjnts[i] = NULL;
		ct->act_jnt_dof[i] = -1;
		ct->act_rob_dof[i] = -1;
		ct->actjnt_state[i] = 1;
		ct->pasjnts[i] = NULL;
		ct->pas_jnt_dof[i] = -1;
		ct->pas_rob_dof[i] = -1;
		ct->argu_i[i] = 0;
		ct->argu_d[i] = 0.0;
		ct->col_pairs[0][i] = NULL;
		ct->col_pairs[1][i] = NULL;
		ct->enchained_rob_dof[i] = -1;
	}
	return ct;
}

/*--------------------------------------------------------------------------*/
/*! \brief Create a generic constraints.
 *
 * \param cntrt_manager:The constraint manager.
 * \param namecntrt:    The name of the constraints.
 * \param nb_passif:    The number of passive joints (-1 to use default values)
 * \param pas_jntPt:    The array of passive joints.
 * \param pas_jnt_dof:  The array of passive degree of freedom in the joints
 * \param pas_rob_dof:  The array of passive degree of freedom in a robot.
 * \param nb_active:    The number of active joints (-1 to use default values)
 * \param act_jntPt:    The array of active joints.
 * \param act_jnt_dof:  The array of active degree of freedom in the joints
 * \param act_rob_dof:  The array of active degree of freedom in a robot.
 *
 * \return The pointer on the new constraint (NULL if there is an error)
 *
 * \internal
 */
p3d_cntrt * p3d_create_generic_cntrts(p3d_cntrt_management * cntrt_manager,
                                      const char *namecntrt, int nb_passif, p3d_jnt ** pas_jntPt,
                                      int * pas_jnt_dof, int * pas_rob_dof, int nb_actif,
                                      p3d_jnt ** act_jntPt, int * act_jnt_dof, int * act_rob_dof) {
	int i, j, active = TRUE;
	p3d_cntrt *ct;
  
	for (i = 0; i < nb_passif; i++) {
		if (cntrt_manager->in_cntrt[pas_rob_dof[i]] == DOF_PASSIF) {
			//       PrintWarning(("ERROR: p3d_create_generic_cntrts: rob_dof is already a passive joint\n"));
			//       return NULL;
			active = FALSE;
		}
	}
	for (i = 0; i < nb_passif; i++) {
		for (j = i + 1; j < nb_passif; j++) {
			if (pas_rob_dof[i] == pas_rob_dof[j]) {
				PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));printf("%s: %d",__FILE__,__LINE__);
				return NULL;
			}
		}
		for (j = 0; j < nb_actif; j++) {
			if (pas_rob_dof[i] == act_rob_dof[j]) {
				PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));printf("%s: %d",__FILE__,__LINE__);
				return NULL;
			}
		}
	}
	for (i = 0; i < nb_actif; i++) {
		for (j = i + 1; j < nb_actif; j++) {
			if (act_rob_dof[i] == act_rob_dof[j]) {
				PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));printf("%s: %d",__FILE__,__LINE__);
				return NULL;
			}
		}
	}
	ct = s_p3d_create_cntrts(cntrt_manager);
	if (ct == NULL) {
		return NULL;
	}
	ct->npasjnts = nb_passif;
	for (i = 0; i < nb_passif; i++) {
		ct->pasjnts[i] = pas_jntPt[i];
		ct->pas_jnt_dof[i] = pas_jnt_dof[i];
		ct->pas_rob_dof[i] = pas_rob_dof[i];
		cntrt_manager->in_cntrt[pas_rob_dof[i]] = DOF_PASSIF;
	}
	ct->nactjnts = nb_actif;
	for (i = 0; i < nb_actif; i++) {
		ct->actjnts[i] = act_jntPt[i];
		ct->act_jnt_dof[i] = act_jnt_dof[i];
		ct->act_rob_dof[i] = act_rob_dof[i];
	}
	strcpy(ct->namecntrt, namecntrt);
	ct->active = active;
	ct->nbSol = 1;
	ct->nSingularities = 0;
	ct->markedForSingularity = 0;
	//   ct->speValSolAssoc = NULL;
	p3d_mat4Copy(p3d_mat4IDENTITY, ct->TSingularity);
	return ct;
}

/* rob_dof = val */
static int p3d_set_fixed_dof(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nbVal,
                             double* val, int ct_num, int state) {
	p3d_cntrt * ct;
  
	//  std::cout << "p3d_setFixed_Joint = " << nbVal << std::endl;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_FIXED_NAME, 1,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_fixed_jnt;
		ct->ndval = nbVal;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	int firstDof = pas_rob_dof[0];
	for (int i = 1; i < nbVal; i++) {
		pas_rob_dof[i] = firstDof+i;
	}
  
	for (int i = 0; i < nbVal; i++) {
		ct->argu_d[i] = val[i];
		//    std::cout << "pas_rob_dof[i] = " << pas_rob_dof[i] << std::endl;
		cntrt_manager->in_cntrt[pas_rob_dof[i]] = DOF_PASSIF;
	}
	//   ct->argu_d[0] = val;
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* rob_dof_pas = k0*rob_dof_act0 + k1*rob_dof_act1 + k2*rob_dof_act2 + ... + kN*rob_dof_actN + alpha*/
static int p3d_set_lin_rel_dofs(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof, int nb_act,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nb_Dval,
                                double* Dval, int ct_num, int state) {
	p3d_cntrt *ct;
	int i;
  
	if (ct_num < 0) {
		/*modif mokhtar adding nb_act/ Before 1*/
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_LIN_REL_NAME, 1,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, nb_act, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_lin_rel_dofs;
		/*modif Mokhtar Before 2 */
		ct->ndval = nb_Dval;
		/*Modif Mokhtar*/
		for (i = 0; i < nb_act; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
    
		/* NOTE: the deactivation or not of the pair could be chosen by
		 the user !!! */
		/*     ct->col_pairs[0][0] = A_jntPt->o; */
		/*     ct->col_pairs[1][0] = B_jntPt->o; */
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
	/*Modif Mokhtar*/
	for (i = 0; i < nb_Dval; i++) {
		ct->argu_d[i] = Dval[i];
	}
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* A = k1 * B * cos C + k2 * B * sin C + k3 * C * cos B + k4 * C * sin B */
static int p3d_set_rel_dofs(p3d_cntrt_management * cntrt_manager,
                            p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                            p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double k1,
                            double k2, double k3, double k4, int ct_num, int state) {
	p3d_cntrt * ct;
	int i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_REL_NAME, 1,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 2, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_rel_dofs;
		ct->ndval = 4;
		for (i = 0; i < 2; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = k1;
	ct->argu_d[1] = k2;
	ct->argu_d[2] = k3;
	ct->argu_d[3] = k4;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* chaine fermee du type RRPR linkage                                       */
/* JO : articulation de rotation du corp a controler                        */
/* JA : articulation de translation. Doit etre place dans le point de tour  */
/*      entre le corp a controler et le corp de longeur variable.           */
/*      C'est l'articulation d'entree du movement                           */
/* JC : articulation de rotation de la basse du corp de longeur variable    */
static int p3d_set_RRPRlnk(p3d_cntrt_management * cntrt_manager,
                           p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                           p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                           int state) {
	p3d_jnt * JO, *JA, *JC;
	p3d_cntrt *ct;
	p3d_vector3 posi_jnt;
	double xO, yO, zO, xA, yA, zA, xC, yC, zC;
	double l_OA, l_CO, l_CA;
	p3d_vector3 v1, v2, v1vectv2, senseOA, senseCA, vaxe;
	double tetamod, chimod;
	int i;
  
	JO = pas_jntPt[0];
	JC = pas_jntPt[1];
	JA = act_jntPt[0];
	if ((JA->type != P3D_TRANSLATE) || (JO->type != P3D_ROTATE) || (JC->type
                                                                  != P3D_ROTATE)) {
		PrintWarning(("ERROR: p3d_set_RRPRlnk: wrong type of joint !!!\n"));
		return FALSE;
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_RRPR_NAME, 2,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_RRPRlnk;
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
    
		i = 0;
		ct->col_pairs[0][i] = JO->o;
		ct->col_pairs[1][i] = JA->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JC->prev_jnt->o;
		ct->col_pairs[1][i] = JA->o;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	p3d_jnt_get_cur_vect_point(JO, posi_jnt);
	xO = posi_jnt[0];
	yO = posi_jnt[1];
	zO = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JA, posi_jnt);
	xA = posi_jnt[0];
	yA = posi_jnt[1];
	zA = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JC, posi_jnt);
	xC = posi_jnt[0];
	yC = posi_jnt[1];
	zC = posi_jnt[2];
  
	l_OA = sqrt(sqr(xA - xO) + sqr(yA - yO) + sqr(zA - zO));
	l_CO = sqrt(sqr(xC - xO) + sqr(yC - yO) + sqr(zC - zO));
	l_CA = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
  
	ct->argu_d[0] = l_OA;
	ct->argu_d[1] = l_CO;
  
	v1[0] = xC - xO;
	v1[1] = yC - yO;
	v1[2] = zC - zO;
	v2[0] = xA - xO;
	v2[1] = yA - yO;
	v2[2] = zA - zO;
	tetamod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                             * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseOA);
  
	p3d_jnt_get_dof_cur_axis(JO, 0, v1vectv2);
  
	p3d_vectNormalize(v1vectv2, vaxe);
	p3d_vectSub(senseOA, vaxe, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001)) {
		tetamod = 180.0 - tetamod;
		ct->argu_i[5] = 2;
	} else {
		ct->argu_i[5] = 1;
	}
  
	v2[0] = xA - xC;
	v2[1] = yA - yC;
	v2[2] = zA - zC;
	chimod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                            * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseCA);
	p3d_vectSub(senseOA, senseCA, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001)) {
		chimod = -chimod;
	}
	if (ct->argu_i[5] == 2) {
		chimod = 180.0 - chimod;
	}
  
	ct->argu_d[7] = tetamod - p3d_jnt_get_dof_deg(JO, 0);
	ct->argu_d[8] = chimod - p3d_jnt_get_dof_deg(JC, 0);
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* chaine fermee du type 4R linkage                                          */
/* JO : articulation de rotation du corp d'entree de mouvement               */
/* JA : articulation de rotation entre le corp d'entree et le corps          */
/*      intermediaire                                                        */
/* JB : articulation de rotation entre le corp intermediaire et le corp de   */
/*      sortie. Placee dans le corp intermediaire                            */
/* JC : articulation de rotation du corp de sortie de mouvement              */
static int p3d_set_4Rlnk(p3d_cntrt_management * cntrt_manager,
                         p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                         p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                         int state) {
	p3d_jnt * JO, *JA, *JB, *JC;
	p3d_cntrt *ct;
	double tetamax, tetamin, calcint;
	p3d_vector3 posi_jnt;
	double xO, yO, zO, xA, yA, zA, xB, yB, zB, xC, yC, zC;
	p3d_matrix3 mat_pl;
	double l_OA, l_AB, l_BC, l_CO;
	p3d_vector3 v1, v2, v1vectv2, senseOA, senseCB, vaxe;
	double tetamod, chimod, fimod;
	int i;
  
	JO = act_jntPt[0];
	JA = pas_jntPt[0];
	JB = pas_jntPt[1];
	JC = pas_jntPt[2];
	if ((JA->type != P3D_ROTATE) || (JO->type != P3D_ROTATE) || (JC->type
                                                               != P3D_ROTATE) || (JB->type != P3D_ROTATE)) {
		PrintWarning(("ERROR: p3d_set_3RPRlnk: wrong type of joint !!!\n"));
		return FALSE;
	}
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_4R_NAME, 3,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_4Rlnk;
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
    
		ct->col_pairs[0][0] = JA->o;
		ct->col_pairs[1][0] = JC->o;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	p3d_jnt_get_cur_vect_point(JO, posi_jnt);
	xO = posi_jnt[0];
	yO = posi_jnt[1];
	zO = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JA, posi_jnt);
	xA = posi_jnt[0];
	yA = posi_jnt[1];
	zA = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JB, posi_jnt);
	xB = posi_jnt[0];
	yB = posi_jnt[1];
	zB = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JC, posi_jnt);
	xC = posi_jnt[0];
	yC = posi_jnt[1];
	zC = posi_jnt[2];
  
	mat_pl[0][0] = xA - xO;
	mat_pl[1][0] = xB - xO;
	mat_pl[2][0] = xC - xO;
	mat_pl[0][1] = yA - yO;
	mat_pl[1][1] = yB - yO;
	mat_pl[2][1] = yC - yO;
	mat_pl[0][2] = zA - zO;
	mat_pl[1][2] = zB - zO;
	mat_pl[2][2] = zC - zO;
  
	l_OA = sqrt(sqr(xA - xO) + sqr(yA - yO) + sqr(zA - zO));
	l_AB = sqrt(sqr(xA - xB) + sqr(yA - yB) + sqr(zA - zB));
	l_BC = sqrt(sqr(xC - xB) + sqr(yC - yB) + sqr(zC - zB));
	l_CO = sqrt(sqr(xC - xO) + sqr(yC - yO) + sqr(zC - zO));
	if ((l_OA > (l_AB + l_BC + l_CO)) || (l_AB > (l_OA + l_BC + l_CO)) || (l_BC
                                                                         > (l_AB + l_OA + l_CO)) || (l_CO > (l_AB + l_BC + l_OA))) {
		PrintWarning((("ERROR: p3d_close_4Rlnk: chain can not be closed : one of the links is too long\n")));
		return (FALSE);
	}
  
	ct->argu_d[0] = l_OA;
	ct->argu_d[1] = l_AB;
	ct->argu_d[2] = l_BC;
	ct->argu_d[3] = l_CO;
  
	calcint = (sqr(l_CO) + sqr(l_OA) - sqr(l_AB - l_BC)) / (2 * l_OA * l_CO);
	if ((calcint >= 1.0) || (calcint <= -1.0))
		tetamin = 23.0;
	else
		tetamin = acos(calcint);
	calcint = (sqr(l_CO) + sqr(l_OA) - sqr(l_AB + l_BC)) / (2 * l_OA * l_CO);
	if ((calcint >= 1.0) || (calcint <= -1.0))
		tetamax = 23.0;
	else
		tetamax = acos(calcint);
  
	ct->argu_d[4] = tetamin; /* in radians !! */
	ct->argu_d[5] = tetamax; /* if the value is 23.0 -> not limit */
  
	v1[0] = xC - xO;
	v1[1] = yC - yO;
	v1[2] = zC - zO;
	v2[0] = xA - xO;
	v2[1] = yA - yO;
	v2[2] = zA - zO;
	tetamod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                             * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseOA);
  
	p3d_jnt_get_dof_cur_axis(JO, 0, v1vectv2);
  
	p3d_vectNormalize(v1vectv2, vaxe);
	p3d_vectSub(senseOA, vaxe, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001)) {
		tetamod = 180.0 - tetamod;
		ct->argu_i[6] = 2;
	} else
		ct->argu_i[6] = 1;
  
	v2[0] = xB - xC;
	v2[1] = yB - yC;
	v2[2] = zB - zC;
	chimod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                            * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseCB);
	p3d_vectSub(senseOA, senseCB, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001))
		chimod = -chimod;
	if (ct->argu_i[6] == 2)
		chimod = 180.0 - chimod;
  
	v1[0] = xA - xO;
	v1[1] = yA - yO;
	v1[2] = zA - zO;
	v2[0] = xB - xA;
	v2[1] = yB - yA;
	v2[2] = zB - zA;
	if (ct->argu_i[6] == 2)
		fimod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                  / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
	else
		fimod = -(180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                   / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
  
	ct->argu_d[7] = tetamod - p3d_jnt_get_dof_deg(JO, 0);
	ct->argu_d[8] = chimod - p3d_jnt_get_dof_deg(JC, 0);
	ct->argu_d[9] = fimod - p3d_jnt_get_dof_deg(JA, 0);
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* << modif EF pour Delmia */
/* chaine fermee du type RRPR linkage mais controlee par JC (JO ?)          */
/* JO : articulation de rotation du corp a controler                        */
/*      c'est le joint de controle                                          */
/* JA : articulation de translation. Doit etre place dans le point de tour  */
/*      entre le corp a controler et le corp de longeur variable.           */
/* JC : articulation de rotation de la basse du corp de longeur variable    */
static int p3d_set_P3Rlnk(p3d_cntrt_management * cntrt_manager,
                          p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                          p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                          int state) {
	p3d_jnt * JO, *JA, *JC;
	p3d_cntrt *ct;
	p3d_vector3 posi_jnt;
	double xO, yO, zO, xA, yA, zA, xC, yC, zC;
	double l_OA, l_CO, l_CA;
	int i;
  
	JC = pas_jntPt[0];
	JA = pas_jntPt[1];
	JO = act_jntPt[0];
	if ((JA->type != P3D_TRANSLATE) || (JO->type != P3D_ROTATE) || (JC->type
                                                                  != P3D_ROTATE)) {
		PrintWarning(("ERROR: p3d_set_RRPRlnk: wrong type of joint !!!\n"));
		return FALSE;
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_P3R_NAME, 2,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_P3Rlnk;
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
    
		i = 0;
		ct->col_pairs[0][i] = JC->o;
		ct->col_pairs[1][i] = JA->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JO->prev_jnt->o;
		ct->col_pairs[1][i] = JA->o;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	p3d_jnt_get_cur_vect_point(JO, posi_jnt);
	xO = posi_jnt[0];
	yO = posi_jnt[1];
	zO = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JA, posi_jnt);
	xA = posi_jnt[0];
	yA = posi_jnt[1];
	zA = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JC, posi_jnt);
	xC = posi_jnt[0];
	yC = posi_jnt[1];
	zC = posi_jnt[2];
  
	l_OA = sqrt(sqr(xA - xO) + sqr(yA - yO) + sqr(zA - zO));
	l_CO = sqrt(sqr(xC - xO) + sqr(yC - yO) + sqr(zC - zO));
	l_CA = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
  
	ct->argu_d[0] = l_OA;
	ct->argu_d[1] = l_CO;
	ct->argu_d[2] = l_CA;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* chaine fermee du type 3RPR linkage */
/* JO : articulation de rotation du corp d'entree de mouvement */
/* JA : articulation de rotation entre le corp d'entree et le corps */
/*      intermediaire */
/* JB : articulation de translation entre le corp intermediaire et le corps */
/*      de sortie. Placee dans le point qui serait la rotation avec le      */
/*      corps intermediaire */
/* JC : articulation de rotation du corp de sortie de mouvement */
static int p3d_set_3RPRlnk(p3d_cntrt_management * cntrt_manager,
                           p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                           p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                           int state) {
	p3d_jnt * JO, *JA, *JB, *JC;
	p3d_cntrt *ct;
	p3d_vector3 posi_jnt;
	double xO, yO, zO, xA, yA, zA, xB, yB, zB, xC, yC, zC;
	p3d_matrix3 mat_pl;
	double l_OA, l_AB, l_BC, l_CO, l_CA;
	p3d_vector3 v1, v2, v1vectv2, senseOA, senseCB, vaxe;
	double tetamod, chimod, fimod;
	int i;
  
	JO = act_jntPt[0];
	JA = pas_jntPt[0];
	JB = act_jntPt[1];
	JC = pas_jntPt[1];
	if ((JA->type != P3D_ROTATE) || (JO->type != P3D_ROTATE) || (JC->type
                                                               != P3D_ROTATE) || (JB->type != P3D_ROTATE)) {
		PrintWarning(("ERROR: p3d_set_3RPRlnk: wrong type of joint !!!\n"));
		return FALSE;
	}
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_3RPR_NAME, 2,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 2, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_3RPRlnk;
		for (i = 0; i < 2; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
    
		i = 0;
		ct->col_pairs[0][i] = JA->o;
		ct->col_pairs[1][i] = JC->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JB->o;
		ct->col_pairs[1][i] = JO->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JA->o;
		ct->col_pairs[1][i] = JB->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JO->o;
		ct->col_pairs[1][i] = JC->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JO->prev_jnt->o;
		ct->col_pairs[1][i] = JA->o;
		if (ct->col_pairs[0][i] != NULL) {
			i++;
		}
		ct->col_pairs[0][i] = JC->prev_jnt->o;
		ct->col_pairs[1][i] = JB->o;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	p3d_jnt_get_cur_vect_point(JO, posi_jnt);
	xO = posi_jnt[0];
	yO = posi_jnt[1];
	zO = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JA, posi_jnt);
	xA = posi_jnt[0];
	yA = posi_jnt[1];
	zA = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JB, posi_jnt);
	xB = posi_jnt[0];
	yB = posi_jnt[1];
	zB = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JC, posi_jnt);
	xC = posi_jnt[0];
	yC = posi_jnt[1];
	zC = posi_jnt[2];
  
	mat_pl[0][0] = xA - xO;
	mat_pl[1][0] = xB - xO;
	mat_pl[2][0] = xC - xO;
	mat_pl[0][1] = yA - yO;
	mat_pl[1][1] = yB - yO;
	mat_pl[2][1] = yC - yO;
	mat_pl[0][2] = zA - zO;
	mat_pl[1][2] = zB - zO;
	mat_pl[2][2] = zC - zO;
	if (p3d_mat3Det(mat_pl) != 0.0) {
		PrintInfo((("ERROR: p3d_set_3RPRlnk: joints must be on the same planar\n")));
		return (FALSE);
	}
  
	l_OA = sqrt(sqr(xA - xO) + sqr(yA - yO) + sqr(zA - zO));
	l_AB = sqrt(sqr(xA - xB) + sqr(yA - yB) + sqr(zA - zB));
	l_BC = sqrt(sqr(xC - xB) + sqr(yC - yB) + sqr(zC - zB));
	l_CO = sqrt(sqr(xC - xO) + sqr(yC - yO) + sqr(zC - zO));
	l_CA = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
	if ((l_OA > (l_AB + l_BC + l_CO)) || (l_AB > (l_OA + l_BC + l_CO)) || (l_BC
                                                                         > (l_AB + l_OA + l_CO)) || (l_CO > (l_AB + l_BC + l_OA))) {
		PrintInfo((("ERROR: p3d_set_3RPRlnk: chain can not be closed : one of the links is too long\n")));
		return (FALSE);
	}
  
	ct->argu_d[0] = l_OA;
	ct->argu_d[1] = l_AB;
	ct->argu_d[2] = l_CA;
	ct->argu_d[3] = l_CO;
  
	v1[0] = xC - xO;
	v1[1] = yC - yO;
	v1[2] = zC - zO;
	v2[0] = xA - xO;
	v2[1] = yA - yO;
	v2[2] = zA - zO;
	tetamod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                             * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseOA);
  
	p3d_jnt_get_dof_cur_axis(JO, 0, vaxe);
	p3d_vectSub(senseOA, vaxe, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001)) {
		tetamod = 180.0 - tetamod;
		ct->argu_i[6] = 2;
	} else
		ct->argu_i[6] = 1;
  
	v2[0] = xB - xC;
	v2[1] = yB - yC;
	v2[2] = zB - zC;
	chimod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                                            * p3d_vectNorm(v2)));
	p3d_vectXprod(v1, v2, v1vectv2);
	p3d_vectNormalize(v1vectv2, senseCB);
	p3d_vectSub(senseOA, senseCB, v1vectv2);
	if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
			|| (fabs(v1vectv2[2]) > 0.0001))
		chimod = -chimod;
	if (ct->argu_i[6] == 2)
		chimod = 180.0 - chimod;
	v1[0] = xA - xO;
	v1[1] = yA - yO;
	v1[2] = zA - zO;
	v2[0] = xB - xA;
	v2[1] = yB - yA;
	v2[2] = zB - zA;
	if (ct->argu_i[6] == 2)
		fimod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                  / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
	else
		fimod = -(180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                   / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
  
	ct->argu_d[7] = tetamod - p3d_jnt_get_dof_deg(JO, 0);
	ct->argu_d[8] = chimod - p3d_jnt_get_dof_deg(JC, 0);
	ct->argu_d[9] = fimod - p3d_jnt_get_dof_deg(JA, 0);
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* glissement d'une articulation dans un plan horizontal                  */
/* Jpasiv[0] : articulation a controler                                   */
/* Jpasiv[i] : articulation bloquee                                       */
/* WARNING ! : je suppose que les vecteurs itab dans rw_env sont init a 0 */
/*             donc je parcours Jpasiv jusqu'a trouver un 0               */
/* Dval[0] : hauteur du plan de glissement                                */
/* Dval[1] : angle entre l'axe forme par J et l'articulation a controler  */
/*            et l'axe z absolut quand le corp est completement suspendu  */
/* Dval[2,3,4] : position du point qui glisse par rapport au jnt controle */
/* Dval[5,...] : valeur de l'articulation bloquee                         */
/* Ival[0] : pour changer le signe de l'angle calcule                     */
/* Ival[1] : pour changer le signe de l'angle calcule en dependant de     */
/*               la situation du corp place avant                         */
/* Ival[2] : pour pas pouvoir soulever                                    */
static int p3d_set_jnt_on_ground(p3d_cntrt_management * cntrt_manager,
                                 int nb_pas, p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                 p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int nb_dval,
                                 double* Dval, int *Ival, int ct_num, int state) {
	p3d_jnt *J;
	p3d_cntrt *ct;
	double dx, dy, dz, distance, tetamod, angcormod, angantmod;
	p3d_vector3 pos0_jnt, posi_jnt;
	int i, rot_axe = 0;
	p3d_vector3 v1, v2, v1vectv2, sense;
  
	J = pas_jntPt[0];
	if (nb_dval != nb_pas + 4) {
		PrintWarning(("ERROR: p3d_set_jnt_on_ground: wrong number "
                  "of float parameters !!!\n"));
		return FALSE;
	}
	if ((J == NULL) || (J->type != P3D_ROTATE)) {
		PrintWarning(("ERROR: p3d_set_jnt_on_ground: wrong type of joint !!!\n"));
		return FALSE;
	}
  
	p3d_jnt_get_dof_axis(J, 0, pos0_jnt);
	if ((pos0_jnt[0] != 1.0) && (pos0_jnt[1] != 1.0)) {
		PrintWarning(("ERROR: p3d_set_jnt_on_ground: wrong type of joint !!!\n"));
		return FALSE;
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_ON_GROUND_NAME, 1,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_jnt_on_ground;
		ct->ndval = 5;
		ct->nival = 3;
    
		if (pos0_jnt[0] == 1.0) {
			rot_axe = 0;
		} else {
			rot_axe = 1;
		}
		ct->argu_i[3] = rot_axe;
    
		/* blocked joints */
		/* NOTE : this part must be out of this if when theated by the interface FORMconstraint */
		for (i = 1; i < nb_pas; i++) {
			ct->pasjnts[i] = pas_jntPt[i];
			/*      ct->pas_rob_dof[i] = pas_rob_dof[i]; */
			/*       ct->pas_jnt_dof[i] = pas_jnt_dof[i]; */
			ct->pas_rob_dof[i] = pas_rob_dof[i];
			ct->pas_jnt_dof[i] = pas_jnt_dof[i];
			ct->argu_d[i+5] = Dval[i+4];
			i++;
		}
		ct->pasjnts[i] = NULL;
    
		/* copy data in Tatt */
		ct->Tatt[0][3] = Dval[2];
		ct->Tatt[1][3] = Dval[3];
		ct->Tatt[2][3] = Dval[4];
    
		dx = ct->Tatt[0][3];
		dy = ct->Tatt[1][3];
		dz = ct->Tatt[2][3];
    
		distance = sqrt(sqr(dx) + sqr(dy) + sqr(dz));
		ct->argu_d[5] = distance;
    
		tetamod = (180.0 / M_PI) * (J->dof_data[0].v);
		ct->argu_d[2] = tetamod;
    
		v1[0] = dx;
		v1[1] = dy;
		v1[2] = dz;
		v2[0] = 0.0;
		v2[1] = 0.0;
		v2[2] = 1.0;
		angcormod = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                      / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
		p3d_vectXprod(v1, v2, v1vectv2);
		p3d_vectNormalize(v1vectv2, sense);
		p3d_jnt_get_dof_cur_axis(J, 0, posi_jnt);
		p3d_vectSub(posi_jnt, sense, v1vectv2);
		if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
				|| (fabs(v1vectv2[2]) > 0.0001))
			angcormod = -angcormod;
		ct->argu_d[3] = angcormod;
    
		angantmod = (180.0 / M_PI) * acos(J->prev_jnt->abs_pos[2][2]);
		if (rot_axe == 0) {
			if (J->prev_jnt->abs_pos[2][1] > 0.0)
				angantmod = -angantmod;
		} else {
			if (J->prev_jnt->abs_pos[2][0] < 0.0)
				angantmod = -angantmod;
		}
		ct->argu_d[4] = angantmod;
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0]; /*z0 */
  
	ct->argu_d[1] = Dval[1]; /* ang_susp */
  
	ct->argu_i[0] = Ival[0]; /* turningsense */
	ct->argu_i[1] = Ival[1]; /* changesense */
	ct->argu_i[2] = Ival[2]; /* stay */
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* tour des roues dans une voiture traite comme hilare avec remorque                        */
/* JO : articulation qui serait le point d'atache de la remorque                            */
/* JR : articulation de la roue droite                                                      */
/* JL : articulation de la roue gauche                                                      */
/* h : distance entre le point d'atache et le centre de la transmision des roues derrireres */
/* a : distance entre les centres des roues d'avant                                         */
static int p3d_set_car_front_wheels(p3d_cntrt_management * cntrt_manager,
                                    p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                    p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double h,
                                    double a, int ct_num, int state) {
	p3d_jnt *JO, *JR, *JL;
	int i_dofJO;
	p3d_cntrt *ct;
	int i;
  
	JO = act_jntPt[0];
	i_dofJO = act_jnt_dof[0];
	JR = pas_jntPt[0];
	JL = pas_jntPt[1];
  
	if ((!p3d_jnt_is_dof_angular(JO, i_dofJO)) || (JR->type != P3D_ROTATE)
			|| (JL->type != P3D_ROTATE)) {
		PrintInfo(("ERROR: p3d_set_car_front_wheels: wrong type of joints\n"));
		return (FALSE);
	}
  
	if ((h < 0.0) || (a < 0.0)) {
		PrintInfo(("ERROR: p3d_set_car_front_wheels: h and a must be positive\n"));
		return (FALSE);
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager,
                                   CNTRT_CAR_FRONT_WHEELS_NAME, 2, pas_jntPt, pas_jnt_dof,
                                   pas_rob_dof, 1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_car_front_wheels;
		ct->ndval = 2;
    
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = h;
	ct->argu_d[1] = a;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}
/* Cycab wheel orientations
 * Jact[0] : steering wheel
 * Jpas[0] : J1 left front wheel
 * Jpas[1] : J2 right front wheel
 * Jpas[2] : J3 left rear wheel
 * Jpas[3] : J4 right rear wheel
 * Dval[0] : l1 distance between projection of curvature center over cycab
 *           longitudinal axis and front wheel axis
 * Dval[1] : l2 distance between projection of curvature center over cycab
 *           longitudinal axis and rear wheel axis
 * Dval[2] : distance between front (or rear) wheels
 */

static int p3d_set_cycab_wheels(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double l1,
                                double l2, double e, int ct_num, int state) {
	p3d_jnt *JO, *J1, *J2, *J3, *J4;
	int i_dofJO;
	p3d_cntrt *ct;
	int i;
  
	JO = act_jntPt[0];
	i_dofJO = act_jnt_dof[0];
	J1 = pas_jntPt[0];
	J2 = pas_jntPt[1];
	J3 = pas_jntPt[2];
	J4 = pas_jntPt[3];
  
	if ((!p3d_jnt_is_dof_angular(JO, i_dofJO)) || (J1->type != P3D_ROTATE)
			|| (J2->type != P3D_ROTATE) || (J3->type != P3D_ROTATE)
			|| (J4->type != P3D_ROTATE)) {
		PrintInfo(("ERROR: p3d_set_cycab_wheels: wrong type of joints\n"));
		return (FALSE);
	}
  
	if ((l1 < 0.0) || (l2 < 0.0) || (e < 0.0)) {
		PrintInfo(("ERROR: p3d_set_cycab_wheels: l1, l2 and e must be positive\n"));
		return (FALSE);
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_CYCAB_WHEELS_NAME,
                                   4, pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_cycab_wheels;
		ct->ndval = 2;
    
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = l1;
	ct->argu_d[1] = l2;
	ct->argu_d[2] = e;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* chaine cinematique planar */
/* JE1 et JE2 sont les points a fermer */
static int p3d_set_planar_closed_chain(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int whatcase,
                                       int ct_num, int state) {
	p3d_jnt * JE1, *JE2, *JB1, *JP1, *JP2;
	p3d_cntrt *ct;
	p3d_vector3 posi_jnt;
	/*   int JP1,JP2; */
	double xE2, yE2, zE2, xE1, yE1, zE1, xP1, yP1, zP1, xP2, yP2, zP2;
	double xref, yref, zref;
	double l_E1P2, l_P2P1, l_E2P1;
	p3d_vector3 v1, v2, v1vectv2, sense, vaxe;
	double angmod;
	int i, j;
  
	JE1 = pas_jntPt[0];
	JE2 = pas_jntPt[1];
	JP1 = pas_jntPt[2];
	JP2 = pas_jntPt[3];
	JB1 = act_jntPt[0];
  
	if ((JE1->type != P3D_ROTATE) || (JE2->type != P3D_ROTATE) || (JB1->type
                                                                 != P3D_ROTATE) || (JP1->type != P3D_ROTATE) || (JP2->type
                                                                                                                 != P3D_ROTATE) || (JE2->prev_jnt->rob != JE2->rob)) {
		PrintWarning(("ERROR: p3d_set_planar_closed_chain: wrong type of joint !!!\n"));
		return FALSE;
	}
  
	xref = yref = zref = 0.0;
  
	p3d_jnt_get_cur_vect_point(JE2, posi_jnt);
	xE2 = posi_jnt[0];
	yE2 = posi_jnt[1];
	zE2 = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JE1, posi_jnt);
	xE1 = posi_jnt[0];
	yE1 = posi_jnt[1];
	zE1 = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JP2, posi_jnt);
	xP2 = posi_jnt[0];
	yP2 = posi_jnt[1];
	zP2 = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JP1, posi_jnt);
	xP1 = posi_jnt[0];
	yP1 = posi_jnt[1];
	zP1 = posi_jnt[2];
  
	l_E1P2 = sqrt(sqr(xE1 - xP2) + sqr(yE1 - yP2) + sqr(zE1 - zP2));
	l_P2P1 = sqrt(sqr(xP2 - xP1) + sqr(yP2 - yP1) + sqr(zP2 - zP1));
	l_E2P1 = sqrt(sqr(xE2 - xP1) + sqr(yE2 - yP1) + sqr(zE2 - zP1));
  
	if (l_E2P1 > l_E1P2 + l_P2P1) {
		PrintInfo(("ERROR: p3d_set_planar_closed_chain: chain must be closable when defined\n"));
		return (FALSE);
	}
  
	if (ct_num == -1) {
		ct = p3d_create_generic_cntrts(cntrt_manager, 
                                   CNTRT_PLANAR_CLOSED_CHAIN_NAME, 4, pas_jntPt, pas_jnt_dof, pas_rob_dof,
                                   1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_planar_closed_chain;
		/*     ct->npasjnts = 4; */
		ct->nival = 1;
    
		ct->col_pairs[0][0] = JE1->prev_jnt->o;
		ct->col_pairs[1][0] = JE2->prev_jnt->o;
    
		/* different treatment for enchaining *//* ESTO FUNCIONA SIEMPRE ????? */
		i = pas_rob_dof[1] - pas_jnt_dof[1] - pas_jntPt[1]->index_dof
    + pas_jntPt[1]->prev_jnt->index_dof;
		for (j = 0; j < pas_jntPt[1]->prev_jnt->dof_equiv_nbr; j++) {
			if (cntrt_manager->in_cntrt[i+j] != DOF_WITHOUT_CNTRT) {
				p3d_enchain_cntrt(ct, i + j, cntrt_manager->in_cntrt[i+j]);
			}
		}
    
		/* Parameters for JP1, JP2, and JE2 */
		/* lengths */
		ct->argu_d[0] = l_P2P1;
		ct->argu_d[1] = l_E1P2;
    
		/* reference angles */
		v1[0] = xref - xP1;
		v1[1] = yref - yP1;
		v1[2] = zref - zP1;
		v2[0] = xP2 - xP1;
		v2[1] = yP2 - yP1;
		v2[2] = zP2 - zP1;
		angmod = acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                             * p3d_vectNorm(v2)));
		if (angmod == 0.0) {
			p3d_vectNormalize(v1, sense);
			p3d_vectNormalize(v2, vaxe);
			p3d_vectSub(sense, vaxe, v1vectv2);
			if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
					|| (fabs(v1vectv2[2]) > 0.0001)) {
				angmod = M_PI;
			}
		} else {
			p3d_vectXprod(v1, v2, v1vectv2);
			p3d_vectNormalize(v1vectv2, sense);
      
			p3d_jnt_get_dof_cur_axis(JP1, 0, vaxe);
			p3d_vectSub(sense, vaxe, v1vectv2);
			if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
					|| (fabs(v1vectv2[2]) > 0.0001)) {
				angmod = -angmod;
			}
		}
		ct->argu_d[3] = (180.0 / M_PI) * (angmod - p3d_jnt_get_dof(JP1, 0)); /* stored in degrees */
    
		v1[0] = -v2[0];
		v1[1] = -v2[1];
		v1[2] = -v2[2];
		v2[0] = xE1 - xP2;
		v2[1] = yE1 - yP2;
		v2[2] = zE1 - zP2;
		angmod = acos(p3d_vectDotProd(v1, v2) / (p3d_vectNorm(v1)
                                             * p3d_vectNorm(v2)));
		if (angmod == 0.0) {
			p3d_vectNormalize(v1, sense);
			p3d_vectNormalize(v2, vaxe);
			p3d_vectSub(sense, vaxe, v1vectv2);
			if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
					|| (fabs(v1vectv2[2]) > 0.0001)) {
				angmod = M_PI;
			}
		} else {
			p3d_vectXprod(v1, v2, v1vectv2);
			p3d_vectNormalize(v1vectv2, sense);
      
			p3d_jnt_get_dof_cur_axis(JP1, 0, vaxe);
			p3d_vectSub(sense, vaxe, v1vectv2);
			if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
					|| (fabs(v1vectv2[2]) > 0.0001)) {
				angmod = -angmod;
			}
		}
		ct->argu_d[4] = (180.0 / M_PI) * (angmod - p3d_jnt_get_dof(JP2, 0)); /* stored in degrees */
    
		/* reference for angle of JP3(==JE2) is not calculated */
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_i[0] = whatcase;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* min rob_dofB = rob_dofA < 0 ? min - rob_dofA : min
 max rob_dofB = rob_dofA < 0 ? max : max - rob_dofA*/
static int p3d_set_min_max_dofs(p3d_cntrt_management * cntrt_manager,
                                p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double min,
                                double max, int ct_num, int state) {
	p3d_cntrt *ct;
	int i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_MIN_MAX_NAME, 0,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 2, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_min_max_dofs;
		ct->ndval = 2;
		for (i = 0; i < 1; i++) {
			if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {
				p3d_enchain_cntrt(ct, act_rob_dof[i],
                          cntrt_manager->in_cntrt[act_rob_dof[i]]);
			} else {
				cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
			}
		}
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = min;
	ct->argu_d[1] = max;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* fonctions pour traiter les contraintes                             */

static int p3d_fct_fixed_jnt(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	/*   p3d_jnt * jntPt; */
	double min, max;
  
	/*   jntPt = ct->pasjnts[0]; */
	/*   while ((jntPt!=NULL) && (jntPt != jntPt->rob->j_modif)) */
	/*     { jntPt = jntPt->prev_jnt; } */
	/*   if(jntPt!=NULL) { */
	for (int i = 0; i < ct->ndval; i++) {
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], i, &min, &max);
		if (ct->argu_d[i] > max || ct->argu_d[i] < min) {
			return (FALSE);
		}
		p3d_jnt_set_dof_deg(ct->pasjnts[0], i, ct->argu_d[i]);
	}
  
	//   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min, &max);
	//   if (ct->argu_d[0] > max) {
	//     return(FALSE);
	//   }
	//   if (ct->argu_d[0] < min) {
	//     return(FALSE);
	//   }
	//   p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], ct->argu_d[0]);
	if (st_niksol) {
		st_niksol[ct->num] = 1;
		st_iksol[ct->num][0] = 1;
		st_ikSolConfig[ct->num][0][0] = ct->argu_d[0];
	}
	/*   } */
	return (TRUE);
}

static int p3d_fct_lin_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double valso = 0, valfo = 0;
	double min, max;
	int i, I_can;
	double lastvalfo;
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	//   if (p3d_go_into_cntrt_fct(ct)) {
  
// 	if (!TEST_PHASE) {
// 		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
// 	}
  
	/*Modif Mokhtar Before :
	 valso = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
	 valfo = valso*ct->argu_d[0] + ct->argu_d[1];*/
	for (i = 0; i < ct->nactjnts; i++) {
		valso = p3d_jnt_get_dof_deg(ct->actjnts[i], ct->act_jnt_dof[0]);
		if (DEBUG_CNTRTS) {
			printf("valso = %f\t%f\n", valso, (valso / 180)*M_PI);
		}
		valfo += valso * ct->argu_d[i];
	}
	valfo += ct->argu_d[ct->ndval-1];
  
	p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min, &max);
	if (valfo - max > EPS6) {
		return (FALSE);
	}
	if (min - valfo > EPS6) {
		return (FALSE);
	}
  
	lastvalfo = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
	p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], valfo);
	if (st_niksol && st_iksol_size > ct->num) {
		st_iksol[ct->num][0] = 1;
		st_niksol[ct->num] = 1;
		st_ikSolConfig[ct->num][0][0] = (valfo / 180) * M_PI;
	}/* else {
    printf("p3d_fct_lin_rel_dof st_iksol ??\n");
    }*/
	if (ct->enchained != NULL) {
		I_can = 1;
		for (i = 0; I_can && (i < ct->nenchained); i++) {
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 1);
			if (ct->enchained[i]->active)
				// multiple iksol is not compatible (yet) with enchained !
				I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i], -1,
                                               qp, dl);
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 0);
		}
		if (!I_can) {
			p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], lastvalfo);
			return (FALSE);
		}
	}
	//   }
	return (TRUE);
}

static int p3d_fct_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double min, max;
	double valJA, valJB, valJC;
	int i, I_can;
	double lastvalJA;
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
		valJB = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
		valJC = p3d_jnt_get_dof_deg(ct->actjnts[1], ct->act_jnt_dof[1]);
		valJA = ct->argu_d[0] * valJB * cos((M_PI / 180.0) * valJC)
    + ct->argu_d[1] * valJB * sin((M_PI / 180.0) * valJC)
    + ct->argu_d[2] * valJC * cos((M_PI / 180.0) * valJB)
    + ct->argu_d[3] * valJC * sin((M_PI / 180.0) * valJB);
		/*    valJA = ct->argu_d[0] * (valJB * cos((M_PI/180.0)*valJC) + valJB); */
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min,
                               &max);
		if (valJA > max) {
			return (FALSE);
		}
		if (valJA < min) {
			return (FALSE);
		}
    
		lastvalJA = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], valJA);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0],
                            lastvalJA);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

static int p3d_fct_RRPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double l_CA;
	double min, max;
	double teta, chi;
	p3d_vector3 posi_jnt;
	double xA, yA, zA, xC, yC, zC;
	int i, I_can;
	double lastvalteta, lastvalchi;
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
    
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);
    
		p3d_jnt_get_cur_vect_point(ct->actjnts[0], posi_jnt);
		xA = posi_jnt[0];
		yA = posi_jnt[1];
		zA = posi_jnt[2];
    
		p3d_jnt_get_cur_vect_point(ct->pasjnts[1], posi_jnt);
		xC = posi_jnt[0];
		yC = posi_jnt[1];
		zC = posi_jnt[2];
		l_CA = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
    
		if (l_CA > (ct->argu_d[0] + ct->argu_d[1])) {
			return (FALSE);
		}
		if (l_CA < (fabs(ct->argu_d[0] - ct->argu_d[1]))) {
			return (FALSE);
		}
    
		if (ct->argu_i[5] == 1) { /* CASE A */
			teta = (180.0 / M_PI) * acos((sqr(ct->argu_d[0]) + sqr(ct->argu_d[1]) - sqr(l_CA)) / (2 * (ct->argu_d[0]) * (ct->argu_d[1])));
			chi = 180.0 - (180.0 / M_PI) * atan2((ct->argu_d[0]) * sin((M_PI
                                                                  / 180.0) * teta), ((ct->argu_d[1]) - (ct->argu_d[0])
                                                                                     * cos((M_PI / 180.0) * teta)));
		} else { /* CASE B */
			teta = -(180.0 / M_PI) * acos((sqr(ct->argu_d[0]) + sqr(ct->argu_d[1]) - sqr(l_CA)) / (2 * (ct->argu_d[0]) * (ct->argu_d[1]))) + 180.0;
			chi = (180.0 / M_PI) * atan2((ct->argu_d[0]) * sin((M_PI / 180.0)
                                                         * teta), ((ct->argu_d[0]) * cos((M_PI / 180.0) * teta)
                                                                   + (ct->argu_d[1])));
		}
    
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min,
                               &max);
		if ((teta - ct->argu_d[7] > max) || (teta - ct->argu_d[7] < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], &min,
                               &max);
		if ((chi - ct->argu_d[8] > max) || (chi - ct->argu_d[8] < min))
			return (FALSE);
    
		lastvalteta = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1]);
		p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], teta
                        - ct->argu_d[7]);
		p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], chi
                        - ct->argu_d[8]);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0],
                            lastvalteta);
				p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1],
                            lastvalchi);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

static int p3d_fct_4Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double min, max;
	double teta, tetamax, tetamin, fi, chi = 0.0;
	double a_te, b_te, c_te;
	int i, I_can;
	double lastvalteta, lastvalchi, lastvalfi;
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
    
		if (ct->argu_i[6] == 1) { /* CASE A */
			teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
			teta += ct->argu_d[7];
			tetamin = (180.0 / M_PI) * ct->argu_d[4] + 0.1;
			tetamax = (180.0 / M_PI) * ct->argu_d[5];
			if (ct->argu_d[5] != 23.0) {
				if (teta > tetamax) {
					return (FALSE);
				} else if ((-teta) > tetamax) {
					return (FALSE);
				}
			}
			if (ct->argu_d[4] != 23.0) {
				if (teta < tetamin) {
					return (FALSE);
				} else if ((360.0 -teta) < tetamin) {
					return (FALSE);
				}
			}
      
			if (teta >= 360.0) {
				teta = 359.9;
			}
			if (teta <= -360.0) {
				teta = -359.9;
			}
      
			a_te = 2 * (ct->argu_d[0]) * (ct->argu_d[2]) * cos((M_PI / 180.0)
                                                         * teta) - 2 * (ct->argu_d[2]) * (ct->argu_d[3]);
			b_te = 2 * (ct->argu_d[0]) * (ct->argu_d[2]) * sin((M_PI / 180.0)
                                                         * teta);
			c_te = sqr(ct->argu_d[3]) + sqr(ct->argu_d[2]) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1]) - 2 * (ct->argu_d[0]) * (ct->argu_d[3]) * cos((M_PI / 180.0) * teta);
      
			if ((b_te == 0.0) && (a_te == 0.0)) {
				if (teta == 0.0)
					chi = 0.0;
				else
					chi = 180.0;
			} else {
				chi = (180.0 / M_PI) * (atan(b_te / a_te) - acos(c_te
                                                         / sqrt(sqr(a_te) + sqr(b_te)))) + 180.0;
			}
      
			fi = (180.0 / M_PI) * atan2(((ct->argu_d[2]) * sin((M_PI / 180.0)
                                                         * chi) - (ct->argu_d[0]) * sin((M_PI / 180.0) * teta)),
                                  ((ct->argu_d[3]) + (ct->argu_d[2]) * cos((M_PI / 180.0)
                                                                           * chi) - (ct->argu_d[0]) * cos((M_PI / 180.0)
                                                                                                          * teta))) - teta;
		} else { /* CASE B */
			teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
			teta += ct->argu_d[7];
      
			tetamax = 180.0 - (180.0 / M_PI) * ct->argu_d[4] - 0.1;
			tetamin = 180.0 - (180.0 / M_PI) * ct->argu_d[5];
			if (ct->argu_d[4] != 23.0) {
				if (teta > tetamax) {
					return (FALSE);
				} else if ((-teta) > tetamax) {
					return (FALSE);
				}
			}
			if (ct->argu_d[5] != 23.0) {
				if (teta < tetamin) {
					return (FALSE);
				} else if ((360.0 -teta) < tetamin) {
					return (FALSE);
				}
			}
      
			if (teta >= 360.0) {
				teta = 359.9;
			}
			if (teta <= -360.0) {
				teta = -359.9;
			}
      
			a_te = 2 * (ct->argu_d[0]) * (ct->argu_d[2]) * cos((M_PI / 180.0)
                                                         * (180.0 - teta)) - 2 * (ct->argu_d[2]) * (ct->argu_d[3]);
			b_te = 2 * (ct->argu_d[0]) * (ct->argu_d[2]) * sin((M_PI / 180.0)
                                                         * (180.0 - teta));
			c_te = sqr(ct->argu_d[3]) + sqr(ct->argu_d[2]) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1]) - 2 * (ct->argu_d[0]) * (ct->argu_d[3]) * cos((M_PI / 180.0) * (180.0 - teta));
      
			if ((b_te == 0.0) && (a_te == 0.0)) {
				if (teta == 0.0)
					chi = 0.0;
				else if (teta == 360.0)
					chi = 180.0;
			} else
				chi = (180.0 / M_PI) * -(atan2(b_te, a_te) - acos(c_te
                                                          / sqrt(sqr(a_te) + sqr(b_te)))) + 180.0;
      
			fi = (180.0 / M_PI) * atan2(((ct->argu_d[2]) * sin((M_PI / 180.0)
                                                         * chi) - (ct->argu_d[0]) * sin((M_PI / 180.0) * teta)),
                                  (-(ct->argu_d[3]) + (ct->argu_d[2]) * cos((M_PI / 180.0)
                                                                            * chi) - (ct->argu_d[0]) * cos((M_PI / 180.0)
                                                                                                           * teta))) - teta;
			if (fi < 0.0)
				fi += 360;
		}
    
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[2], ct->pas_jnt_dof[2], &min,
                               &max);
		if ((chi - ct->argu_d[8] > max) || (chi - ct->argu_d[8] < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min,
                               &max);
		if ((fi - ct->argu_d[9] > max) || (fi - ct->argu_d[9] < min))
			return (FALSE);
    
		lastvalteta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
		lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[2], ct->pas_jnt_dof[2]);
		lastvalfi = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0], teta
                        - ct->argu_d[7]);
		p3d_jnt_set_dof_deg(ct->pasjnts[2], ct->pas_jnt_dof[2], chi
                        - ct->argu_d[8]);
		p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], fi
                        - ct->argu_d[9]);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                            lastvalteta);
				p3d_jnt_set_dof_deg(ct->pasjnts[2], ct->pas_jnt_dof[2],
                            lastvalchi);
				p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0],
                            lastvalfi);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

/* << modif EF pour Delmia */
static int p3d_fct_P3Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double l_AO, l_OC, l_AC, dx;
	double min, max;
	double theta, phi;
	p3d_vector3 posi_jnt;
	p3d_matrix4 inv_pos0_O, rel_pos_OA, pos_A_in_O;
	double xA, yA, zA, xC, yC, zC;
	int i, I_can;
	double lastvaldx, lastvalphi;
  
	if (p3d_go_into_cntrt_fct(ct)) {
    
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);
    
		p3d_matInvertXform(ct->actjnts[0]->pos0, inv_pos0_O);
		p3d_matMultXform(inv_pos0_O, ct->pasjnts[1]->pos0, rel_pos_OA);
		/* Position of A needed */
		p3d_matMultXform(ct->actjnts[0]->abs_pos, rel_pos_OA, pos_A_in_O);
		xA = pos_A_in_O[0][3];
		yA = pos_A_in_O[1][3];
		zA = pos_A_in_O[2][3];
    
		p3d_jnt_get_cur_vect_point(ct->pasjnts[0], posi_jnt);
		xC = posi_jnt[0];
		yC = posi_jnt[1];
		zC = posi_jnt[2];
    
		l_AC = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
    
		dx = (l_AC - ct->argu_d[2]);
		p3d_jnt_get_dof_bounds(ct->pasjnts[1], ct->pas_jnt_dof[1], &min, &max);
		if ((dx > max) || (dx < min))
			return (FALSE);
    
		l_OC = ct->argu_d[1];
		l_AO = ct->argu_d[0];
		theta = p3d_jnt_get_dof(ct->actjnts[0], ct->act_jnt_dof[0]);
		phi = -atan2(l_OC * sin(theta), l_OC * cos(theta) - l_AO) + theta
    + M_PI;
		phi = angle_limit_PI(phi);
		p3d_jnt_get_dof_bounds(ct->pasjnts[0], ct->pas_jnt_dof[0], &min, &max);
		if ((phi > max) || (phi < min))
			return (FALSE);
    
		lastvalphi = p3d_jnt_get_dof(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		lastvaldx = p3d_jnt_get_dof(ct->pasjnts[1], ct->pas_jnt_dof[1]);
		p3d_jnt_set_dof(ct->pasjnts[0], ct->pas_jnt_dof[0], phi);
		p3d_jnt_set_dof(ct->pasjnts[1], ct->pas_jnt_dof[1], dx);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof(ct->pasjnts[0], ct->pas_jnt_dof[0], lastvalphi);
				p3d_jnt_set_dof(ct->pasjnts[1], ct->pas_jnt_dof[1], lastvaldx);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

static int p3d_fct_3RPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double min, max;
	double teta, tetamax, tetamin, fi, chi = 0.0;
	double a_te, b_te, c_te;
	double xA, yA, zA, xB, yB, zB, xC, yC, zC;
	double l_BC, l_AC;
	p3d_jnt * JO, *JA, *JB, *JC;
	p3d_vector3 posi_jnt;
	double tetamax_r, tetamin_r, calcint;
	int i, I_can;
	double lastvalteta, lastvalchi, lastvalfi;
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
    
		JO = ct->actjnts[0];
		JA = ct->pasjnts[0];
		JB = ct->actjnts[1];
		JC = ct->pasjnts[1];
    
		/*    update_one_jnt_pos(r->joints[JO]); */
		p3d_update_this_robot_pos_without_cntrt_and_obj(JO->rob);
    
		p3d_jnt_get_cur_vect_point(JA, posi_jnt);
		xA = posi_jnt[0];
		yA = posi_jnt[1];
		zA = posi_jnt[2];
		p3d_jnt_get_cur_vect_point(JB, posi_jnt);
		xB = posi_jnt[0];
		yB = posi_jnt[1];
		zB = posi_jnt[2];
		p3d_jnt_get_cur_vect_point(JC, posi_jnt);
		xC = posi_jnt[0];
		yC = posi_jnt[1];
		zC = posi_jnt[2];
    
		l_BC = sqrt(sqr(xC - xB) + sqr(yC - yB) + sqr(zC - zB));
		l_AC = sqrt(sqr(xC - xA) + sqr(yC - yA) + sqr(zC - zA));
    
		if (l_BC > (ct->argu_d[2] + ct->argu_d[1])) {
			return (FALSE);
		} else if (l_BC < (l_AC - ct->argu_d[1])) {
			return (FALSE);
		}
    
		calcint = (sqr(ct->argu_d[3]) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1] - l_BC)) / (2 * ct->argu_d[0] * ct->argu_d[3]);
		if ((calcint >= 1.0) || (calcint <= -1.0))
			tetamin_r = 23.0;
		else
			tetamin_r = acos(calcint);
		calcint = (sqr(ct->argu_d[3]) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1] + l_BC)) / (2 * ct->argu_d[0] * ct->argu_d[3]);
		if ((calcint >= 1.0) || (calcint <= -1.0))
			tetamax_r = 23.0;
		else
			tetamax_r = acos(calcint);
    
		if (ct->argu_i[6] == 1) { /* CASE A */
			teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
			teta += ct->argu_d[7];
      
			tetamin = (180.0 / M_PI) * tetamin_r + 0.1;
			tetamax = (180.0 / M_PI) * tetamax_r;
			if (tetamax_r != 23.0) {
				if (teta > tetamax) {
					return (FALSE);
				} else if ((-teta) > tetamax) {
					return (FALSE);
				}
			}
			if (tetamin_r != 23.0) {
				if (teta < tetamin) {
					return (FALSE);
				} else if ((360.0 -teta) < tetamin) {
					return (FALSE);
				}
			}
      
			if (teta >= 360.0) {
				teta = 359.9;
			}
			if (teta <= -360.0) {
				teta = -359.9;
			}
      
			a_te = 2 * (ct->argu_d[0]) * (l_BC) * cos((M_PI / 180.0) * teta)
      - 2 * (l_BC) * (ct->argu_d[3]);
			b_te = 2 * (ct->argu_d[0]) * (l_BC) * sin((M_PI / 180.0) * teta);
			c_te = sqr(ct->argu_d[3]) + sqr(l_BC) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1]) - 2 * (ct->argu_d[0]) * (ct->argu_d[3]) * cos((M_PI / 180.0) * teta);
      
			if ((b_te == 0.0) && (a_te == 0.0)) {
				if (teta == 0.0)
					chi = 0.0;
				else
					chi = 180.0;
			} else {
				chi = (180.0 / M_PI) * (atan(b_te / a_te) - acos(c_te
                                                         / sqrt(sqr(a_te) + sqr(b_te)))) + 180.0;
			}
      
			fi = (180.0 / M_PI) * atan2(((l_BC) * sin((M_PI / 180.0) * chi)
                                   - (ct->argu_d[0]) * sin((M_PI / 180.0) * teta)),
                                  ((ct->argu_d[3]) + (l_BC) * cos((M_PI / 180.0) * chi)
                                   - (ct->argu_d[0]) * cos((M_PI / 180.0) * teta)))
      - teta;
		} else { /* CASE B */
			teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
			teta += ct->argu_d[7];
      
			tetamax = 180.0 - (180.0 / M_PI) * tetamin_r - 0.1;
			tetamin = 180.0 - (180.0 / M_PI) * tetamax_r;
			if (tetamin_r != 23.0) {
				if (teta > tetamax) {
					return (FALSE);
				} else if ((-teta) > tetamax) {
					return (FALSE);
				}
			}
			if (tetamax_r != 23.0) {
				if (teta < tetamin) {
					return (FALSE);
				} else if ((360.0 -teta) < tetamin) {
					return (FALSE);
				}
			}
      
			if (teta >= 360.0) {
				teta = 359.9;
			}
			if (teta <= -360.0) {
				teta = -359.9;
			}
      
			a_te = 2 * (ct->argu_d[0]) * (l_BC) * cos((M_PI / 180.0) * (180.0
                                                                  - teta)) - 2 * (l_BC) * (ct->argu_d[3]);
			b_te = 2 * (ct->argu_d[0]) * (l_BC) * sin((M_PI / 180.0) * (180.0
                                                                  - teta));
			c_te = sqr(ct->argu_d[3]) + sqr(l_BC) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1]) - 2 * (ct->argu_d[0]) * (ct->argu_d[3]) * cos((M_PI / 180.0) * (180.0 - teta));
      
			if ((b_te == 0.0) && (a_te == 0.0)) {
				if (teta == 0.0)
					chi = 0.0;
				else if (teta == 360.0)
					chi = 180.0;
			} else
				chi = (180.0 / M_PI) * -(atan2(b_te, a_te) - acos(c_te
                                                          / sqrt(sqr(a_te) + sqr(b_te)))) + 180.0;
      
			fi = (180.0 / M_PI) * atan2(((l_BC) * sin((M_PI / 180.0) * chi)
                                   - (ct->argu_d[0]) * sin((M_PI / 180.0) * teta)),
                                  (-(ct->argu_d[3]) + (l_BC) * cos((M_PI / 180.0) * chi)
                                   - (ct->argu_d[0]) * cos((M_PI / 180.0) * teta)))
      - teta;
			if (fi < 0.0)
				fi += 360;
		}
    
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], &min,
                               &max);
		if ((chi - ct->argu_d[8] > max) || (chi - ct->argu_d[8] < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min,
                               &max);
		if ((fi - ct->argu_d[9] > max) || (fi - ct->argu_d[9] < min))
			return (FALSE);
    
		lastvalteta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
		lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1]);
		lastvalfi = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0], teta
                        - ct->argu_d[7]);
		p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], chi
                        - ct->argu_d[8]);
		p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], fi
                        - ct->argu_d[9]);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                            lastvalteta);
				p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1],
                            lastvalchi);
				p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0],
                            lastvalfi);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

static int p3d_fct_jnt_on_ground(p3d_cntrt *ct, int iksol, configPt qp,
                                 double dl) {
	double ang, teta_ant, value;
	p3d_vector4 posi_jnt;
	double xJ, yJ, zJ, xJA, yJA, zJA;
	static int f_t = 1, sens;
	pp3d_jnt J, JA;
	int i;
  
	J = ct->pasjnts[0];
	JA = ct->pasjnts[0]->prev_jnt;
  
	/*   if(r->joints[ct->pasjnts[0]]->pos[2][ct->argu_i[3]] == 0.0) { */
  
	p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob);
  
	p3d_jnt_get_cur_vect_point(J, posi_jnt);
	xJ = posi_jnt[0];
	yJ = posi_jnt[1];
	zJ = posi_jnt[2];
  
	teta_ant = (180.0 / M_PI) * acos(JA->abs_pos[2][2]);
	if (ct->argu_i[3] == 0) {
		if (JA->abs_pos[2][1] > 0.0)
			teta_ant = -teta_ant;
	} else {
		if (JA->abs_pos[2][0] < 0.0)
			teta_ant = -teta_ant;
	}
  
	if (zJ > (ct->argu_d[5] + ct->argu_d[0])) {
		if (ct->argu_i[2] == 1)
			return (FALSE);
		else
			ang = ct->argu_d[1];
	} else {
		ang = (180.0 / M_PI) * acos((zJ - ct->argu_d[0]) / ct->argu_d[5]);
		/* blocked joints */
		i = 1;
		while (ct->pasjnts[i] != NULL) {
			p3d_jnt_set_dof_deg(ct->pasjnts[i], ct->pas_jnt_dof[i],
                          ct->argu_d[i+5]);
			i++;
		}
	}
  
	if ((ct->argu_i[1] == 1)
			&& (f_t || (zJ >= (ct->argu_d[5] + ct->argu_d[0])))) {
		p3d_jnt_get_cur_vect_point(JA, posi_jnt);
		xJA = posi_jnt[0];
		yJA = posi_jnt[1];
		zJA = posi_jnt[2];
		if ((xJ > xJA) || (yJ > yJA))
			sens = 0;
		else
			sens = 1;
		f_t = 0;
	}
  
	if (ct->argu_i[1] == 0) {
		if (ct->argu_i[0] == 0) {
			value = 180.0 - ang + teta_ant + ct->argu_d[2] + ct->argu_d[3]
      - ct->argu_d[4];
		} else {
			value = - 180.0 + ang + teta_ant + ct->argu_d[2] + ct->argu_d[3]
      - ct->argu_d[4];
		}
	} else {
		if (sens) {
			value = 180.0 - ang + teta_ant + ct->argu_d[2] + ct->argu_d[3]
      - ct->argu_d[4];
		} else {
			value = - 180.0 + ang + teta_ant + ct->argu_d[2] + ct->argu_d[3]
      - ct->argu_d[4];
		}
	}
	if (value > 360.0)
		value -= 360;
	else if (value < -360.0)
		value += 360;
  
	p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], value);
  
	/* cette contrainte n'est pas preparee pour avoir des contraites enchainees */
  
	return (TRUE);
	/*   } */
	/*   else { */
	/*     PrintInfo(("ERROR: p3d_lowest_z_const : robot can not use this constraint\n"));  */
	/*     return(FALSE); */
	/*   } */
}

static int p3d_fct_car_front_wheels(p3d_cntrt *ct, int iksol, configPt qp,
                                    double dl) {
	double theta0, thetaR, thetaL, d;
	double min, max;
	int i, I_can;
	double lastvalR, lastvalL;
	static configPt q = NULL;
  
	if (q == NULL) {
		q = p3d_alloc_body_config();
	}
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
		if (ct->actjnts[0]->num == 0) {
			p3d_get_robot_pos_deg(q);
			theta0 = q[3];
		} else
			theta0 = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
		if (theta0 == 0.0) {
			thetaR = 0.0;
			thetaL = 0.0;
		} else {
			d = ct->argu_d[0] / fabs(tan((M_PI / 180.0) * theta0));
			if (theta0 < 0.0) {
				thetaR = (180.0 / M_PI) * atan2(ct->argu_d[0], d
                                        + (ct->argu_d[1]) / 2);
				thetaL = (180.0 / M_PI) * atan2(ct->argu_d[0], d
                                        - (ct->argu_d[1]) / 2);
			} else {
				thetaR = -(180.0 / M_PI) * atan2(ct->argu_d[0], d
                                         - (ct->argu_d[1]) / 2);
				thetaL = -(180.0 / M_PI) * atan2(ct->argu_d[0], d
                                         + (ct->argu_d[1]) / 2);
			}
		}
    
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], &min,
                               &max);
		if ((thetaR > max) || (thetaR < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], &min,
                               &max);
		if ((thetaL > max) || (thetaL < min))
			return (FALSE);
    
		lastvalR = p3d_jnt_get_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		lastvalL = p3d_jnt_get_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1]);
		p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], thetaR);
		p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1], thetaL);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0],
                            lastvalR);
				p3d_jnt_set_dof_deg(ct->pasjnts[1], ct->pas_jnt_dof[1],
                            lastvalL);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

static int p3d_fct_cycab_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double d, phi, phi1, phi2, phi3, phi4;
	double min, max;
	int i, I_can;
	double lastvalPhi1, lastvalPhi2, lastvalPhi3, lastvalPhi4;
	double l1 = ct->argu_d[0];
	double l2 = ct->argu_d[1];
	double e = ct->argu_d[2];
	static configPt q = NULL;
  
	if (q == NULL) {
		q = p3d_alloc_body_config();
	}
  
	/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
	if (p3d_go_into_cntrt_fct(ct)) {
		if (ct->actjnts[0]->num == 0) {
			p3d_get_robot_pos(q);
			phi = q[3];
		} else
			phi = p3d_jnt_get_dof(ct->actjnts[0], ct->act_jnt_dof[0]);
		if (phi == 0.0) {
			phi1 = 0.0;
			phi2 = 0.0;
			phi3 = 0.0;
			phi4 = 0.0;
		} else {
			d = l1 / tan(phi);
      
			phi1 = atan(l1 / (d - e / 2));
			phi2 = atan(l1 / (d + e / 2));
			phi3 = -atan(l2 / (d - e / 2));
			phi4 = -atan(l2 / (d + e / 2));
		}
    
		p3d_jnt_get_dof_bounds(ct->pasjnts[0], ct->pas_jnt_dof[0], &min, &max);
		if ((phi1 > max) || (phi1 < min))
			return (FALSE);
    
		p3d_jnt_get_dof_bounds(ct->pasjnts[1], ct->pas_jnt_dof[1], &min, &max);
		if ((phi2 > max) || (phi2 < min))
			return (FALSE);
    
		p3d_jnt_get_dof_bounds(ct->pasjnts[2], ct->pas_jnt_dof[2], &min, &max);
		if ((phi3 > max) || (phi4 < min))
			return (FALSE);
    
		p3d_jnt_get_dof_bounds(ct->pasjnts[3], ct->pas_jnt_dof[3], &min, &max);
		if ((phi4 > max) || (phi4 < min))
			return (FALSE);
    
		lastvalPhi1 = p3d_jnt_get_dof(ct->pasjnts[0], ct->pas_jnt_dof[0]);
		lastvalPhi2 = p3d_jnt_get_dof(ct->pasjnts[1], ct->pas_jnt_dof[1]);
		lastvalPhi3 = p3d_jnt_get_dof(ct->pasjnts[2], ct->pas_jnt_dof[2]);
		lastvalPhi4 = p3d_jnt_get_dof(ct->pasjnts[3], ct->pas_jnt_dof[3]);
    
		p3d_jnt_set_dof(ct->pasjnts[0], ct->pas_jnt_dof[0], phi1);
		p3d_jnt_set_dof(ct->pasjnts[1], ct->pas_jnt_dof[1], phi2);
		p3d_jnt_set_dof(ct->pasjnts[2], ct->pas_jnt_dof[2], phi3);
		p3d_jnt_set_dof(ct->pasjnts[3], ct->pas_jnt_dof[3], phi4);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof(ct->pasjnts[0], ct->pas_jnt_dof[0], lastvalPhi1);
				p3d_jnt_set_dof(ct->pasjnts[1], ct->pas_jnt_dof[1], lastvalPhi2);
				p3d_jnt_set_dof(ct->pasjnts[2], ct->pas_jnt_dof[2], lastvalPhi3);
				p3d_jnt_set_dof(ct->pasjnts[3], ct->pas_jnt_dof[3], lastvalPhi4);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

/* ------------------------------------------------------------------ */

static int p3d_fct_planar_closed_chain(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl) {
	p3d_jnt *JP1, *JP2, *JE2, *Jref;
	double min, max;
	double xP1, yP1, zP1, xE2, yE2, zE2, xref, yref, zref;
	double distance, angACy, theta1, theta2, theta3, chi;
	double lasttheta1, lasttheta2, lasttheta3;
	p3d_vector3 posi_jnt;
	p3d_vector3 v1, v2, v1vectv2, sense;
	int i, I_can;
  
	JP1 = ct->pasjnts[2];
	JP2 = ct->pasjnts[3];
	JE2 = ct->pasjnts[1];
  
	/*  if(j->num != 0) { */
	p3d_update_this_robot_pos_without_cntrt_and_obj(JP1->rob);
	/*  } */
  
	if ((JP1->prev_jnt->type == P3D_TRANSLATE) && ((JP1->prev_jnt->p0.x
                                                  == JP1->p0.x) && (JP1->prev_jnt->p0.y == JP1->p0.y)
                                                 && (JP1->prev_jnt->p0.z == JP1->p0.z)))
		Jref = JP1->prev_jnt->prev_jnt;
	else
		Jref = JP1->prev_jnt;
  
	p3d_jnt_get_cur_vect_point(JE2, posi_jnt);
	xE2 = posi_jnt[0];
	yE2 = posi_jnt[1];
	zE2 = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(JP1, posi_jnt);
	xP1 = posi_jnt[0];
	yP1 = posi_jnt[1];
	zP1 = posi_jnt[2];
	p3d_jnt_get_cur_vect_point(Jref, posi_jnt);
	xref = posi_jnt[0];
	yref = posi_jnt[1];
	zref = posi_jnt[2];
	distance = sqrt(sqr(xP1 - xE2) + sqr(yP1 - yE2) + sqr(zP1 - zE2));
  
	/*  PrintInfo(("%f\n",distance)); */
	if ((distance - 0.01 > (ct->argu_d[0] + ct->argu_d[1])) || (distance
                                                              < fabs(ct->argu_d[0] - ct->argu_d[1]))) {
		/*  PrintInfo(("%f\n",distance)); */
		/*    PrintInfo(("FALSE\n"));   */
		return (FALSE);
	} else {
		v1[0] = xref - xP1;
		v1[1] = yref - yP1;
		v1[2] = zref - zP1;
		v2[0] = xE2 - xP1;
		v2[1] = yE2 - yP1;
		v2[2] = zE2 - zP1;
		angACy = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                   / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
		p3d_vectXprod(v1, v2, v1vectv2);
		p3d_vectNormalize(v1vectv2, sense);
    
		p3d_jnt_get_dof_cur_axis(JP1, 0, posi_jnt);
		p3d_vectSub(posi_jnt, sense, v1vectv2);
		if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
				|| (fabs(v1vectv2[2]) > 0.0001))
			angACy = -angACy;
    
		theta1 = (180.0 / M_PI) * acos((sqr(distance) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1])) / (2 * distance * ct->argu_d[0]));
		theta2 = (180.0 / M_PI) * asin(ct->argu_d[0] * sin((M_PI / 180.0)
                                                       * theta1) / ct->argu_d[1]);
    
		p3d_jnt_get_cur_vect_point(JE2->prev_jnt, posi_jnt);
		xref = posi_jnt[0];
		yref = posi_jnt[1];
		zref = posi_jnt[2];
		v2[0] = -v2[0];
		v2[1] = -v2[1];
		v2[2] = -v2[2];
		v1[0] = xE2 - xref;
		v1[1] = yE2 - yref;
		v1[2] = zE2 - zref;
		theta3 = (180.0 / M_PI) * acos(p3d_vectDotProd(v1, v2)
                                   / (p3d_vectNorm(v1) * p3d_vectNorm(v2)));
		p3d_vectXprod(v1, v2, v1vectv2);
		p3d_vectNormalize(v1vectv2, sense);
    
		p3d_jnt_get_dof_cur_axis(JE2, 0, posi_jnt);
		p3d_vectSub(posi_jnt, sense, v1vectv2);
		if ((fabs(v1vectv2[0]) > 0.0001) || (fabs(v1vectv2[1]) > 0.0001)
				|| (fabs(v1vectv2[2]) > 0.0001))
			theta3 = -theta3;
    
		if ((ct->argu_d[0]*cos((M_PI / 180.0)*theta1)) > distance)
			theta2 = 180.0 - theta2;
    
		chi = 180.0 - (theta1 + theta2);
    
		if (ct->argu_i[0] == 0) {
			theta3 = theta3 + theta2;
			theta1 = -theta1 + angACy - ct->argu_d[3];
			theta2 = -chi - ct->argu_d[4];
		} else {
			theta3 = theta3 - theta2;
			theta1 = theta1 + angACy - ct->argu_d[3];
			theta2 = chi - ct->argu_d[4];
		}
    
		p3d_jnt_get_dof_bounds_deg(JP1, 0, &min, &max);
		if ((min >= -180) && (max <= 180)) {
			if (theta1 < -180)
				theta1 += 360;
			else if (theta1 > 180)
				theta1 -= 360;
		}
		if ((theta1 > max) || (theta1 < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(JP2, 0, &min, &max);
		if ((min >= -180) && (max <= 180)) {
			if (theta2 < -180)
				theta2 += 360;
			else if (theta2 > 180)
				theta2 -= 360;
		}
		if ((theta2 > max) || (theta2 < min))
			return (FALSE);
		p3d_jnt_get_dof_bounds_deg(JE2, 0, &min, &max);
		if ((min >= -180) && (max <= 180)) {
			if (theta3 < -180)
				theta3 += 360;
			else if (theta3 > 180)
				theta3 -= 360;
		}
		if ((theta3 > max) || (theta3 < min))
			return (FALSE);
    
		lasttheta1 = p3d_jnt_get_dof_deg(JP1, 0);
		lasttheta2 = p3d_jnt_get_dof_deg(JP2, 0);
		lasttheta3 = p3d_jnt_get_dof_deg(JE2, 0);
		p3d_jnt_set_dof_deg(JP1, 0, theta1);
		p3d_jnt_set_dof_deg(JP2, 0, theta2);
		p3d_jnt_set_dof_deg(JE2, 0, theta3);
    
		if (ct->enchained != NULL) {
			I_can = 1;
			for (i = 0; I_can && (i < ct->nenchained); i++) {
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 1);
				if (ct->enchained[i]->active)
					// multiple iksol is not compatible (yet) with enchained !
					I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],
                                                 -1, qp, dl);
				p3d_change_act_rob_dof_state(ct->enchained[i],
                                     ct->enchained_rob_dof[i], 0);
			}
			if (!I_can) {
				p3d_jnt_set_dof_deg(JP1, 0, lasttheta1);
				p3d_jnt_set_dof_deg(JP2, 0, lasttheta2);
				p3d_jnt_set_dof_deg(JE2, 0, lasttheta3);
				return (FALSE);
			}
		}
	}
	return (TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for IK (of 3R arm) -- */
static int p3d_set_3R_arm_ik(p3d_cntrt_management * cntrt_manager, int nb_pas,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                             int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_matrix4 invT;
	//   int i = 0, j = 0;
  
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_3R_ARM_NAME, 3,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_3R_arm_ik;
		ct->ndval = 5; /*???????????????????*/
		ct->nival = 1; /*???????????????????*/
    
		ct->col_pairs[0][0] = ct->actjnts[0]->o;
		ct->col_pairs[1][0] = ct->pasjnts[2]->o;
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0]; /* r1 */
	ct->argu_d[1] = Dval[1]; /* r2 */
	ct->argu_d[2] = Dval[2]; /* gtoq3 */
  
	//   ct->nSingularities = 1;
	//   ct->specificJnt[0] = (int) Dval[3]; //the passive joint
	//   ct->specificValues[0] = MY_ALLOC(double, 1);
	//   ct->specificValues[0][0] = Dval[4]; //the value to have a singularity
	ct->nbSol = 2;
	//alloc the Specific Value / Solution  matrix
	//   ct->/*speValSolAssoc*/ = MY_ALLOC(int*, ct->nbSol);
	//   for(i = 0; i < ct->nbSol; i++){
	//     ct->speValSolAssoc[i] = MY_ALLOC(int, ct->nbSol);
	//     for(j = 0; j < ct->nbSol; j++){
	//       ct->speValSolAssoc[i][j] = -1;//initialisation
	//     }
	//   }
	//   ct->speValSolAssoc[0][1] = 0; //set the correspondance
  
	/* max. extension (do not consider joint bounds !!!) */
	ct->argu_d[MAX_ARGU_CNTRT - 1] = Dval[0] + Dval[1] + Dval[2];
	/* min. extension (do not consider joint bounds !!!) */
	if (Dval[0] > Dval[1] + Dval[2])
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[0] - Dval[1] + Dval[2];
	else if (Dval[1] > Dval[0] + Dval[2])
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[1] - Dval[0] + Dval[2];
	else if (Dval[2] > Dval[0] + Dval[1])
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[2] - Dval[0] + Dval[1];
	else
		ct->argu_d[MAX_ARGU_CNTRT - 2] = 0.0;
  
	ct->argu_i[0] = Ival[0]; /* up/down */
  
	/* transformation between the preceding joint and the base jnt */
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->pos0, invT);
	p3d_mat4Mult(invT, ct->pasjnts[0]->abs_pos, ct->Tbase);
  
	ct->TSingularity[1][3] += 15; //The offset matrix
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
  
	last_cntrt_set = ct;
	return (TRUE);
  
}

static void p3d_3R_arm_ik(p3d_cntrt *ct, double theta1, double theta2,
                          double angref, p3d_vector3 z_axis, p3d_matrix4 Tgrip, double q[3]) {
	p3d_vector3 l2_axis, l3_axis, vprod;
	double theta3;
  
	theta1 += angref;
  
	l2_axis[0] = -sin(theta1 + theta2);
	l2_axis[1] = cos(theta1 + theta2);
	l2_axis[2] = 0;
  
	l3_axis[0] = Tgrip[0][1];
	l3_axis[1] = Tgrip[1][1];
	l3_axis[2] = Tgrip[2][1];
  
	theta3 = acos(p3d_vectDotProd(l2_axis, l3_axis) / (p3d_vectNorm(l2_axis)
                                                     * p3d_vectNorm(l3_axis)));
	p3d_vectXprod(l2_axis, l3_axis, vprod);
	if (!p3d_same_sign_vect(vprod, z_axis))
		theta3 = -theta3;
	if (DEBUG_CNTRTS) {
		printf("theta1 = %f\n", theta1);
		printf("theta2 = %f\n", theta2);
		printf("theta3 = %f\n", theta3);
	}
	q[0] = theta1;
	q[1] = theta2;
	q[2] = theta3;
  
}

static int p3d_fct_3R_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	p3d_rob *r;
	double min, max;
	int i, j, k = 0, ikChoice = p3d_get_ik_choice();
	p3d_matrix4 Tbase, Tgrip, inv_jnt_mat;
	p3d_matrix4 gripTJ3, TJ3/*, Tatt*/;
	double distance;
	double **q, qlast[3];
	p3d_vector3 posJ1, posJ3;
	p3d_vector3 pos_diff, J1J3, vprod;
	p3d_vector3 y_axis = { 0, 1, 0 };
	p3d_vector3 z_axis = { 0, 0, 1 };
  
	double distJ1J3;
	double angref, theta1, theta2;
  
	r = ct->pasjnts[0]->rob;
	p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  
	q = MY_ALLOC(double*, 2);
	for (i = 0; i < 2; i++) {
		q[i] = MY_ALLOC(double, 3);
	}
  
	p3d_mat4Copy(ct->actjnts[0]->abs_pos, Tbase);
	p3d_mat4Mult(Tbase, ct->Tatt, Tgrip);
	p3d_matInvertXform(ct->pasjnts[0]->jnt_mat, inv_jnt_mat);
	p3d_mat4Mult(ct->pasjnts[0]->abs_pos, inv_jnt_mat, Tbase);
  
	distance = sqrt(sqr(Tgrip[0][3] - Tbase[0][3]) + sqr(Tgrip[1][3] - Tbase[1][3]) + sqr(Tgrip[2][3] - Tbase[2][3]));
	if (DEBUG_CNTRTS) {
		printf("distance = %f, D1 = %f, D2 = %f\n", distance,
           ct->argu_d[MAX_ARGU_CNTRT - 1], ct->argu_d[MAX_ARGU_CNTRT - 2]);
	}
  
	// check if end-frame is in workspace
	if ((distance > ct->argu_d[MAX_ARGU_CNTRT - 1]) || (distance
                                                      < ct->argu_d[MAX_ARGU_CNTRT - 2])) {
		for (i = 0; i < 2; i++) {
			MY_FREE(q[i], double, 3);
		}
		MY_FREE(q, double*, 2);
		return (FALSE);
		//return(TRUE);
	}
  
	// WARNING : suppose that robot in home position is aligned with y-axis (-> REFERENCES)
	p3d_mat4Copy(p3d_mat4IDENTITY, gripTJ3);
	gripTJ3[1][3] = -ct->argu_d[2];
	p3d_mat4Mult(Tgrip, gripTJ3, TJ3);
  
	posJ1[0] = Tbase[0][3];
	posJ1[1] = Tbase[1][3];
	posJ1[2] = Tbase[2][3];
	posJ3[0] = TJ3[0][3];
	posJ3[1] = TJ3[1][3];
	posJ3[2] = TJ3[2][3];
  
	p3d_vectSub(posJ3, posJ1, pos_diff);
	distJ1J3 = p3d_vectNorm(pos_diff);
	p3d_vectNormalize(pos_diff, J1J3);
	angref = acos(p3d_vectDotProd(y_axis, J1J3));
	p3d_vectXprod(y_axis, J1J3, vprod);
	if (!p3d_same_sign_vect(vprod, z_axis))
		angref = -angref;
  
	theta1 = acos((sqr(distJ1J3) + sqr(ct->argu_d[0]) - sqr(ct->argu_d[1])) / (2 * distJ1J3 * ct->argu_d[0]));
	theta2 = acos((sqr(ct->argu_d[0]) + sqr(ct->argu_d[1]) - sqr(distJ1J3)) / (2 * ct->argu_d[0] * ct->argu_d[1])) - M_PI;
	if (iksol != -1) {//if the iksol is specified do like IK_NORMAL
		ikChoice = IK_NORMAL;
	}
	switch (ikChoice) {
    case IK_NORMAL: {
      if (iksol != -1) {
        if (iksol == 2) {//second solution like ct->argu_i[0] == -1
          theta1 = -theta1;
          theta2 = -theta2;
        }
      } else {
        if (ct->argu_i[0] == -1) {
          iksol = 2;
          theta1 = -theta1;
          theta2 = -theta2;
        } else {
          iksol = 1;
        }
      }
      p3d_3R_arm_ik(ct, theta1, theta2, angref, z_axis, Tgrip, q[0]);
      // check joint ranges
      //TODO a rendre générique
      for (i = 0; i < ct->npasjnts; i++) {
        p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i], &min,
                               &max);
        qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
        if ((q[0][i] <= max) && (q[0][i] >= min)) {
          p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[0][i]);
        } else {
          for (j = 0; j < i; j++) {
            p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                            qlast[j]);
          }
          for (i = 0; i < 2; i++) {
            MY_FREE(q[i], double, 3);
          }
          MY_FREE(q, double*, 2);
          return (FALSE);
        }
      }
      if (i == ct->npasjnts) {//if all joints bounds are ok
        st_niksol[ct->num] = 1;
        st_iksol[ct->num][0] = iksol;
        for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
          st_ikSolConfig[ct->num][0][j] = q[0][j];
        }
      }
      break;
    }
    case IK_UNIQUE: {
      iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
      if (iksol == 2) {//second solution like ct->argu_i[0] == -1
        theta1 = -theta1;
        theta2 = -theta2;
      }
      p3d_3R_arm_ik(ct, theta1, theta2, angref, z_axis, Tgrip, q[0]);
      // check joint ranges
      //TODO a rendre générique
      for (i = 0; i < ct->npasjnts; i++) {
        p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i], &min,
                               &max);
        qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
        if ((q[0][i] <= max) && (q[0][i] >= min)) {
          p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[0][i]);
        } else {
          for (j = 0; j < i; j++) {
            p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                            qlast[j]);
          }
          for (i = 0; i < 2; i++) {
            MY_FREE(q[i], double, 3);
          }
          MY_FREE(q, double*, 2);
          return (FALSE);
        }
      }
      if (i == ct->npasjnts) {//if all joints bounds are ok
        st_niksol[ct->num] = 1;
        st_iksol[ct->num][0] = iksol;
        for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
          st_ikSolConfig[ct->num][0][j] = q[0][j];
        }
      }
      break;
    }
    case IK_MULTISOL: {// sol 1 = arg = 1 & sol 2 = arg = -1
      p3d_3R_arm_ik(ct, theta1, theta2, angref, z_axis, Tgrip, q[0]);
      p3d_3R_arm_ik(ct, -theta1, -theta2, angref, z_axis, Tgrip, q[1]);
      for (k = 1; k >= 0; k--) {
        for (i = 0; i < ct->npasjnts; i++) {
          p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                                 &min, &max);
          qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
          if ((q[k][i] <= max) && (q[k][i] >= min)) {
            p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[k][i]);
          } else {
            for (j = 0; j < i; j++) {
              p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                              qlast[j]);
            }
            for (i = 0; i < 2; i++) {
              MY_FREE(q[i], double, 3);
            }
            MY_FREE(q, double*, 2);
            return (FALSE);//faux (on espere que le bras soit symetrique car on sort de la boucle avant de passer en revue toutes les solutions)
          }
        }
        if (i == ct->npasjnts) {//if all joints bounds are ok
          st_niksol[ct->num] = 2;
          for (i = 0; i <= 1; i++) {//for each solution
            st_iksol[ct->num][i] = i + 1;
            for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
              st_ikSolConfig[ct->num][i][j] = q[i][j];
            }
          }
        }
      }
      break;
    }
	}
	for (i = 0; i < 2; i++) {
		MY_FREE(q[i], double, 3);
	}
	MY_FREE(q, double*, 2);
	return (TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for IK (of R6arm) -- */
static int p3d_set_R6_arm_ik(p3d_cntrt_management * cntrt_manager, int nb_pas,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                             int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_jnt * jnt_arrPt[6];
	int jnt_dof_arrPt[6];
	int rob_dof_arrPt[6];
	p3d_jnt * jnt_prevPt, * jntPt;
	int i;
	p3d_matrix4 invT;
  
	if (nb_pas == 6) {
		for (i = 0; i < 6; i++) {
			if (pas_jntPt[i]->type != P3D_ROTATE) {
				return FALSE;
			}
			jnt_arrPt[i] = pas_jntPt[i];
			jnt_dof_arrPt[i] = pas_jnt_dof[i];
			rob_dof_arrPt[i] = pas_rob_dof[i];
		}
	} else {
		jntPt = jnt_prevPt = pas_jntPt[0];
		for (i = 0; i < 6; i++) {
			if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob)
					|| (jntPt->type != P3D_ROTATE) || ((i < 5)
                                             && (jntPt->n_next_jnt != 1))) {
            return FALSE;
          }
			jnt_arrPt[i] = jntPt;
			jnt_dof_arrPt[i] = 0;
			rob_dof_arrPt[i] = pas_rob_dof[0] + jntPt->index_dof
      - jnt_arrPt[0]->index_dof;
      
			jnt_prevPt = jntPt;
			if (jntPt->next_jnt == NULL)
				jntPt = NULL;
			else
				jntPt = jntPt->next_jnt[0];
		}
	}
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_R6_ARM_NAME, 6,
                                   jnt_arrPt, jnt_dof_arrPt, rob_dof_arrPt, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_R6_arm_ik;
		ct->ndval = 3; /*Te length between the principals rotations*/
		ct->nival = 3; /*The kinematic class*/
    
		ct->col_pairs[0][0] = ct->actjnts[0]->o;
		ct->col_pairs[1][0] = ct->pasjnts[5]->o;
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0]; /* a2 */
	ct->argu_d[1] = Dval[1]; /* r4 */
	ct->argu_d[2] = Dval[2]; /* gtoq6 */
  
	/* max. extension (do not consider joint bounds !!!) */
	ct->argu_d[MAX_ARGU_CNTRT - 1] = Dval[0] + Dval[1] + Dval[2];
	/* min. extension (do not consider joint bounds !!!) */
	if (Dval[0] > Dval[1] + Dval[2]) {
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[0] - Dval[1] + Dval[2];
	} else if (Dval[1] > Dval[0] + Dval[2]) {
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[1] - Dval[0] + Dval[2];
	} else if (Dval[2] > Dval[0] + Dval[1]) {
		ct->argu_d[MAX_ARGU_CNTRT - 2] = Dval[2] - Dval[0] + Dval[1];
	} else {
		ct->argu_d[MAX_ARGU_CNTRT - 2] = 0.0;
	}
	//matrix of solutions
	/*
	 Sol 1 = 1 1 1
	 Sol 2 = 1 1 -1
	 Sol 3 = 1 -1 1
	 Sol 4 = 1 -1 -1
	 Sol 5 = -1 1 1
	 Sol 6 = -1 1 -1
	 Sol 7 = -1 -1 1
	 Sol 8 = -1 -1 -1
	 */
	switch (Ival[0]) {
    case - 1: {
      ct->argu_i[0] = Ival[0];
      ct->argu_i[1] = Ival[1];
      ct->argu_i[2] = Ival[2];
      break;
    }
    case 1: {
      ct->argu_i[0] = 1;
      ct->argu_i[1] = 1;
      ct->argu_i[2] = 1;
      break;
    }
    case 2: {
      ct->argu_i[0] = 1;
      ct->argu_i[1] = 1;
      ct->argu_i[2] = -1;
      break;
    }
    case 3: {
      ct->argu_i[0] = 1;
      ct->argu_i[1] = -1;
      ct->argu_i[2] = 1;
      break;
    }
    case 4: {
      ct->argu_i[0] = 1;
      ct->argu_i[1] = -1;
      ct->argu_i[2] = -1;
      break;
    }
    case 5: {
      ct->argu_i[0] = -1;
      ct->argu_i[1] = 1;
      ct->argu_i[2] = 1;
      break;
    }
    case 6: {
      ct->argu_i[0] = -1;
      ct->argu_i[1] = 1;
      ct->argu_i[2] = -1;
      break;
    }
    case 7: {
      ct->argu_i[0] = -1;
      ct->argu_i[1] = -1;
      ct->argu_i[2] = 1;
      break;
    }
    case 8: {
      ct->argu_i[0] = -1;
      ct->argu_i[1] = -1;
      ct->argu_i[2] = -1;
      break;
    }
    default: {
      return FALSE;
    }
	}
	/* transformation between the preceding joint and the base jnt */
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->pos0, invT);
	p3d_mat4Mult(invT, ct->pasjnts[0]->abs_pos, ct->Tbase);
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	ct->nbSol = 8;
	last_cntrt_set = ct;
	return (TRUE);
  
}

static int p3d_fct_R6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	p3d_rob *r = NULL;
	double min = 0.0, max = 0.0;
	int i = 0, j = 0, k = 0, ikChoice = p3d_get_ik_choice(), solution[8][3];
	p3d_matrix4 Tbase, Tgrip, inv_jnt_mat/*, tmp*/;
	double distance = 0.0;
	double **q = NULL, qlast[6];
  
	r = ct->pasjnts[0]->rob;
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
		/* necesario ???????? *//* solo si no es en generacion ??????? */
	}
  
	//matrix of solutions
	/*
	 Sol 1 = 1 1 1
	 Sol 2 = 1 1 -1
	 Sol 3 = 1 -1 1
	 Sol 4 = 1 -1 -1
	 Sol 5 = -1 1 1
	 Sol 6 = -1 1 -1
	 Sol 7 = -1 -1 1
	 Sol 8 = -1 -1 -1
	 */
	solution[0][0] = 1;
	solution[0][1] = 1;
	solution[0][2] = 1;
	solution[1][0] = 1;
	solution[1][1] = 1;
	solution[1][2] = -1;
	solution[2][0] = 1;
	solution[2][1] = -1;
	solution[2][2] = 1;
	solution[3][0] = 1;
	solution[3][1] = -1;
	solution[3][2] = -1;
	solution[4][0] = -1;
	solution[4][1] = 1;
	solution[4][2] = 1;
	solution[5][0] = -1;
	solution[5][1] = 1;
	solution[5][2] = -1;
	solution[6][0] = -1;
	solution[6][1] = -1;
	solution[6][2] = 1;
	solution[7][0] = -1;
	solution[7][1] = -1;
	solution[7][2] = -1;
  
	q = MY_ALLOC(double*, 8);
	for (i = 0; i < 8; i++) {
		q[i] = MY_ALLOC(double, 6);
	}
  
	p3d_mat4Copy(ct->actjnts[0]->abs_pos, Tbase);
	p3d_mat4Mult(Tbase, ct->Tatt, Tgrip);
	p3d_matInvertXform(ct->pasjnts[0]->jnt_mat, inv_jnt_mat);
	p3d_mat4Mult(ct->pasjnts[0]->abs_pos, inv_jnt_mat, Tbase);
  
	//   //set the transformation between the arm and the object
	//   p3d_matInvertXform(ct->Tatt, tmp);
	//   p3d_mat4Copy(tmp, ct->TSingularity);
  
	distance = sqrt(sqr(Tgrip[0][3] - Tbase[0][3]) + sqr(Tgrip[1][3] - Tbase[1][3]) + sqr(Tgrip[2][3] - Tbase[2][3]));
  
	/* FALTA : */
	/* - controlar limites */
	/* - referencias angulares (generalizar) */
	st_niksol[ct->num] = 0;
	if (iksol != -1) {//if the iksol is specified do like IK_NORMAL
		ikChoice = IK_NORMAL;
	}
	if (ikChoice != IK_MULTISOL) {
		if (ikChoice == IK_UNIQUE) {
			iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
		}
		if (ikChoice == IK_NORMAL && iksol == -1) {
			iksol = ct->argu_i[0];
		}
		if (!compute_inverse_kinematics_R6_arm(q[0], Tgrip, Tbase,
                                           ct->argu_d[0], ct->argu_d[1], ct->argu_d[2],
                                           solution[iksol-1][0], solution[iksol-1][1],
                                           solution[iksol-1][2])) {
			for (i = 0; i < 8; i++) {
				if (q[i] != NULL) {
					MY_FREE(q[i], double, 6);
				}
			}
			MY_FREE(q, double*, 8);
			q = NULL;
			return (FALSE);
		} else {
			for (i = 0; i < ct->npasjnts; i++) {
				p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                               &min, &max);
				qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
				if ((q[0][i] <= max) && (q[0][i] >= min)) {
					p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[0][i]);
				} else {
					for (j = 0; j < i; j++) {
						p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                            qlast[j]);
					}
					for (i = 0; i < 8; i++) {
						if (q[i] != NULL) {
							MY_FREE(q[i], double, 6);
						}
					}
					MY_FREE(q, double*, 8);
					q = NULL;
					return (FALSE);
				}
			}
			if (i == ct->npasjnts) {//if all joints bounds are ok
				st_niksol[ct->num] = 1;
				st_iksol[ct->num][0] = iksol;
				for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
					st_ikSolConfig[ct->num][0][j] = q[0][j];
				}
			}
		}
	} else {// Case multisol
		if (!compute_all_ik_R6_arm(q, Tgrip, Tbase, ct->argu_d[0],
                               ct->argu_d[1], ct->argu_d[2])) {
			for (i = 0; i < 8; i++) {
				if (q[i] != NULL) {
					MY_FREE(q[i], double, 6);
				}
			}
			MY_FREE(q, double*, 8);
			q = NULL;
			return FALSE; //There is no solutions
		} else {//verify joint bounds
			for (k = 0; k < 8; k++) {
				if (q[k] != NULL) {//it's a valid solution
					for (i = 0; i < ct->npasjnts; i++) {
						p3d_jnt_get_dof_bounds(ct->pasjnts[i],
                                   ct->pas_jnt_dof[i], &min, &max);
						qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i],
                                       ct->pas_jnt_dof[i]);
						if ((q[k][i] <= max) && (q[k][i] >= min)) {
							p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i],
                              q[k][i]);
						} else {
							for (j = 0; j < i; j++) {
								p3d_jnt_set_dof(ct->pasjnts[j],
                                ct->pas_jnt_dof[j], qlast[j]);
							}
							break;
						}
					}
					if (i == ct->npasjnts) {//if all joints bounds are ok
						st_niksol[ct->num]
            = st_niksol[ct->num] >= 0 ? st_niksol[ct->num]
            + 1 : 1;//there is another valid solution
						st_iksol[ct->num][st_niksol[ct->num] - 1] = k + 1; //set the number of the solution
						for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
							st_ikSolConfig[ct->num][st_niksol[ct->num] - 1][j]
              = q[k][j];
						}
					}
				}
			}
		}
	}
  
	for (i = 0; i < 8; i++) {
		if (q[i] != NULL) {
			MY_FREE(q[i], double, 6);
		}
	}
	MY_FREE(q, double*, 8);
	q = NULL;
	return (TRUE);
  
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for IK (of KUKA R7 ARM) -- */
static int p3d_set_kuka_arm_ik(p3d_cntrt_management * cntrt_manager,
                               p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                               p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                               double * dVal, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_matrix4 r0Base;
	int nb_act = 1, i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_KUKA_ARM_IK_NAME,
                                   6, pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_kuka_arm_ik;
		ct->nival = 3;
		ct->ndval = 0;
		ct->nbSol = 8;//This constraint have a maximum of 8 solutions.
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_i[0] = iVal[0]; //the fixed joint
	ct->argu_i[1] = iVal[1]; //the arm left or right
	ct->argu_i[2] = iVal[2]; // Ik solution
  
	if (iVal[2] > ct->nbSol) { //if the user don't put a valid solution number
		return FALSE;
	}
  
	//Transformation between torso and the arm
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->prev_jnt->pos0, r0Base); //inverse matrix R0->base
	p3d_mat4Mult(r0Base, ct->pasjnts[0]->prev_jnt->pos0, ct->Tbase); //store the transform matrix betweeen torso and the arm base
  
	for (i = 0; i < nb_act; i++) {
		if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {// if a active Dof Is a passiv Dof for another constraint enchain.
			p3d_enchain_cntrt(ct, act_rob_dof[i],
                        cntrt_manager->in_cntrt[act_rob_dof[i]]);
		} else {
			cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
		}
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
  
}

#if defined(USE_GBM)
/* -- functions for IK (of LWR ARM) -- */
static int p3d_set_lwr_arm_ik(p3d_cntrt_management * cntrt_manager,
                              p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                              p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                              double * dVal, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_matrix4 r0Base;
	int nb_act = 1, i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_LWR_ARM_IK_NAME,
                                   6, pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_lwr_arm_ik;
		//ct->nival = 3;
		ct->nival = 2;
		ct->ndval = 0;
		ct->nbSol = 8;//This constraint has a maximum of 8 solutions.
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_i[0] = iVal[0]; //the fixed joint
	ct->argu_i[1] = iVal[1]; // Ik solution
  
	if (iVal[1] > ct->nbSol) { //if the user don't put a valid solution number
		return FALSE;
	}
  
	//Transformation between torso and the arm
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->prev_jnt->pos0, r0Base); //inverse matrix R0->base
	p3d_mat4Mult(r0Base, ct->pasjnts[0]->prev_jnt->pos0, ct->Tbase); //store the transform matrix betweeen torso and the arm base
  
	for (i = 0; i < nb_act; i++) {
		if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {// if a active Dof Is a passiv Dof for another constraint enchain.
			p3d_enchain_cntrt(ct, act_rob_dof[i],
                        cntrt_manager->in_cntrt[act_rob_dof[i]]);
		} else {
			cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
		}
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
  
}

/* -- functions for IK (of LWR ARM) -- */
static int p3d_set_pr2_arm_ik(p3d_cntrt_management * cntrt_manager,
                              p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                              p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                              double * dVal, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_matrix4 r0Base;
	int nb_act = 1, i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_PR2_ARM_IK_NAME,
                                   6, pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_pr2_arm_ik;
		ct->nival = 2;
		ct->ndval = 0;
		ct->nbSol = 8;//This constraint has a maximum of 8 solutions.
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_i[0] = iVal[0]; //the fixed joint
	ct->argu_i[1] = iVal[1]; //Ik solution
  
	if (iVal[1] > ct->nbSol) { //if the user don't put a valid solution number
		return FALSE;
	}
  
	//Transformation between torso and the arm
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->pos0, r0Base); //inverse matrix R0->base
	p3d_mat4Mult(r0Base, ct->pasjnts[0]->pos0, ct->Tbase); //store the transform matrix betweeen torso and the arm base
  
	for (i = 0; i < nb_act; i++) {
		if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {// if a active Dof Is a passiv Dof for another constraint enchain.
			p3d_enchain_cntrt(ct, act_rob_dof[i],
                        cntrt_manager->in_cntrt[act_rob_dof[i]]);
		} else {
			cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
		}
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
  
}

#endif

/** Check if we don't go beyond the bounds of passive joints for all possible constraint
 * solutions
 * @param solutions[][] all configurations generated by the ik
 * @param valid[] array of valid solutions
 * @param minmax[][] the upper and lower limit for each joint
 * @return the number of valid solutions
 */
static int p3d_valid_solutions(p3d_cntrt *ct, double solutions[8][7],
                               int valid[8], double minmax[6][2], int ctNum) {
	int i = 0, j = 0, nbValid = 0, k = 0;
	for (i = 0; i < ct->nbSol; i++) {
		if (valid[i] == 1) {
			for (j = 0; j < 7; j++) {
				if (j != 2) {
					k = j < 2 ? j : j - 1;
					if (solutions[i][j] < minmax[k][0] || solutions[i][j]
							> minmax[k][1]) {
						break;
					}
				}
			}
			if (j == 7) {
				st_iksol[ctNum][nbValid] = i + 1; //the number of the valid solution
				for (j = 0; j < 7; j++) {
					if (j != 2) {
						k = j < 2 ? j : j - 1;
						if (DEBUG_CNTRTS) {
							printf("joint = %d v = %f: min = %f, max = %f\n",
                     j, solutions[i][j], minmax[k][0],
                     minmax[k][1]);
						}
						st_ikSolConfig[ctNum][nbValid][k] = solutions[i][j];
					}
				}
				nbValid++;
				if (DEBUG_CNTRTS) {
					printf("solution : %d\n", i);
					printf("\n");
				}
			}
		}
	}
	if (st_niksol)
		st_niksol[ctNum] = nbValid;
	return nbValid;
}

/** Check if we don't go beyond the bounds of passive joints. If there is enchained constraint
 *
 * @param q[] the configuration of the arm
 * @param ct the constraint
 * @param qp the configuration pointer of the robot
 * @param dl
 * @return true if q is on the bounds false otherwise
 */
static int p3d_check_joints_bounds(double q[7], p3d_cntrt *ct, configPt qp,
                                   double dl) {
	int i = 0, j = 0, k = 0, iCan;
	double qlast[7], min, max;
	for (i = 0; i < 7; i++) {
		if (i != 2) {//the fixed joint
			j = i < 2 ? i : i - 1;
			p3d_jnt_get_dof_bounds(ct->pasjnts[j], ct->pas_jnt_dof[j], &min,
                             &max);
			qlast[i] = p3d_jnt_get_dof(ct->pasjnts[j], ct->pas_jnt_dof[j]);
			if ((q[i] <= max) && (q[i] >= min)) {
				p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j], q[i]);
			} else {
				for (j = 0; j < i; j++) {
					if (j != 2) {
						k = j < 2 ? j : j - 1;
						p3d_jnt_set_dof(ct->pasjnts[k], ct->pas_jnt_dof[k],
                            qlast[j]);
					}
				}
				if (DEBUG_CNTRTS) {
					printf(
                 "out of bounds %d, min = %f, max = %f, qlast = %f, q = %f\n\n",
                 i, min, max, qlast[i], q[i]);
				}
				if (st_niksol)
					st_niksol[ct->num] = 0;
				return FALSE;
			}
		}
	}
	if (DEBUG_CNTRTS) {
		printf("CtNum = %d\n", ct->num);
		for (i = 0; i < 7; i++) {
			printf("q[%d] = %f\n", i, q[i]*180 / M_PI);
		}
		printf("\n");
	}
	if (ct->enchained != NULL) {
		iCan = 1;
		for (i = 0; iCan && (i < ct->nenchained); i++) {
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 1);
			if (ct->enchained[i]->active)
				// multiple iksol is not compatible (yet) with enchained !
				iCan = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i], -1, qp,
                                              dl);
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 0);
		}
		if (!iCan) {
			for (j = 0; j < 7; j++) {
				if (j != 2) {
					k = j < 2 ? j : j - 1;
					p3d_jnt_set_dof(ct->pasjnts[k], ct->pas_jnt_dof[k],
                          qlast[j]);
				}
			}
			return (FALSE);
		}
	}
	return TRUE;
}

/** Compute the inverse kinematic of justin Arm. You can use a specific solution
 * of the ik given in the constraint declaration (the p3d file) or use multi solutions
 *
 * @param ct the constraint calling this function
 * @param iksol the solution of the ik (given by the planner)
 * @param qp the configuration pointer of the robot
 * @param dl
 * @return true if the function succeed false otherwise
 */
static int p3d_fct_kuka_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	int i = 0, j = 0, valid[8], ikChoice = p3d_get_ik_choice(),
  nbSolutions = 0, scale = 1;
	p3d_matrix4 r0Arm, armR0, armGrip, tmp;
	p3d_jnt * fixed;
	double q[7], alphaArray[7], dArray[7], thetaArray[7], qm[8][7],
  minmax[6][2];
	if (ct->pasjnts[0]->o->BB0.xmax - ct->pasjnts[0]->o->BB0.xmin < 1) {//the robot is defind in meters
		scale = 1;
	} else {//the robot is defind in millimeters
		scale = 1000;
	}
	//DH paramerters
	double aArray[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	if (ct->argu_i[1] == -1) {//left Arm
		alphaArray[0] = M_PI;
		alphaArray[1] = M_PI / 2;
		alphaArray[2] = -M_PI / 2;
		alphaArray[3] = M_PI / 2;
		alphaArray[4] = -M_PI / 2;
		alphaArray[5] = -M_PI / 2;
		alphaArray[6] = -M_PI / 2;
		dArray[0] = 0.0;
		dArray[1] = 0.0;
		dArray[2] = -0.400 * scale;
		dArray[3] = 0.0;
		dArray[4] = -0.390 * scale;
		dArray[5] = 0.0;
		dArray[6] = 0.0;
		thetaArray[0] = 0.0;
		thetaArray[1] = 0.0;
		thetaArray[2] = -M_PI / 2;
		thetaArray[3] = 0.0;
		thetaArray[4] = 0.0;
		thetaArray[5] = M_PI / 2;
		thetaArray[6] = -M_PI / 2;
	} else { // right Arm
		alphaArray[0] = 0.0;
		alphaArray[1] = M_PI / 2;
		alphaArray[2] = -M_PI / 2;
		alphaArray[3] = M_PI / 2;
		alphaArray[4] = M_PI / 2;
		alphaArray[5] = -M_PI / 2;
		alphaArray[6] = M_PI / 2;
		dArray[0] = 0.0;
		dArray[1] = 0.0;
		dArray[2] = 0.400 * scale;
		dArray[3] = 0.0;
		dArray[4] = 0.390 * scale;
		dArray[5] = 0.0;
		dArray[6] = 0.0;
		thetaArray[0] = 0.0;
		thetaArray[1] = 0.0;
		thetaArray[2] = -M_PI / 2;
		thetaArray[3] = 0.0;
		thetaArray[4] = -M_PI;
		thetaArray[5] = M_PI / 2;
		thetaArray[6] = -M_PI / 2;
	}
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
	}
  
	fixed = ct->pasjnts[0]->rob->joints[ct->argu_i[0]]; //get the fixed joint
  
	//Position of the grip in the arm Base
	p3d_mat4Mult(ct->pasjnts[0]->prev_jnt->prev_jnt->abs_pos, ct->Tbase, r0Arm);
	p3d_matInvertXform(r0Arm, armR0);
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, tmp);
	p3d_mat4Mult(armR0, tmp, armGrip);
  
	if (DEBUG_CNTRTS) {
		p3d_mat4Print(armGrip, "armGrip");
		printf("fixed joint value = %f\n", fixed->dof_data[0].v);
	}
  
	if (iksol != -1) {
		ikChoice = IK_NORMAL;
	}
	if (ikChoice == IK_NORMAL || ikChoice == IK_UNIQUE) {
		if (ikChoice == IK_UNIQUE) {
			iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
		} else if (ikChoice == IK_NORMAL) {
			iksol = iksol != -1 ? iksol : ct->argu_i[2];
		}
		switch (ikKUKAArmSolverUnique(fixed->dof_data[0].v, aArray, alphaArray,
                                  dArray, thetaArray, armGrip, q, ct->argu_i[1], iksol)) {
      case 0: {
        if (DEBUG_CNTRTS)
          printf("IK KUKA not valid\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case -1: {
        if (DEBUG_CNTRTS)
          printf("IK KUKA not valid, incorrect parameters\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case -2: {
        if (DEBUG_CNTRTS)
          printf("Singularity joint 2\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case -3: {
        if (DEBUG_CNTRTS)
          printf("Singularity joint 0\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case -4: {
        if (DEBUG_CNTRTS)
          printf("Singularity joint 3\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      default: {
        if (DEBUG_CNTRTS) {
          printf("jnt");
          for (i = 0; i < 7; i++) {
            printf("q[%d] = %f\n", i, q[i]);
          }
        }
        if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
          if (DEBUG_CNTRTS) {
            //             for(i = 1; i < 9; i++){
            //               printf("test for sol = %d\n", i);
            //               if(ikKUKAArmSolverUnique(fixed->dof_data[0].v,aArray,alphaArray,dArray,thetaArray, armGrip,
            //                             q, ct->argu_i[1],i) == 1){
            //                 if(p3d_check_joints_bounds(q, ct, qp, dl)){
            //                   printf("Solution %d valide\n", i);
            //                 }else{
            //                   printf("Solution %d invalide\n", i);
            //                 }
            //               }
            //             }
          }
          return (FALSE);
        }
        if (st_niksol) {
          st_niksol[ct->num] = 1;
          st_iksol[ct->num][0] = iksol;
          for (i = 0; i < 7; i++) {
            if (i != 2) {//the fixed joint
              j = i < 2 ? i : i - 1;
              st_ikSolConfig[ct->num][0][j] = q[i];
            }
          }
        }
        return (TRUE);
      }
		}
	} else if (ikChoice == IK_MULTISOL) {
		ikKUKAArmSolver(fixed->dof_data[0].v, aArray, alphaArray, dArray,
                    thetaArray, armGrip, qm, valid, ct->argu_i[1]);
		if (DEBUG_CNTRTS) {
			for (i = 1; i <= 8; i++) {
				printf("solution: %d, %d\n", i, ikKUKAArmSolverUnique(
                                                              fixed->dof_data[0].v, aArray, alphaArray, dArray,
                                                              thetaArray, armGrip, q, ct->argu_i[1], i));
			}
		}
		for (i = 0; i < 6; i++) {
			p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                             &minmax[i][0], &minmax[i][1]);
		}
		//     p3d_jnt_get_dof_bounds(fixed,0, &minmax[2][0], &minmax[2][1]);
		if ((nbSolutions = p3d_valid_solutions(ct, qm, valid, minmax, ct->num))
				== 0)
			return FALSE; //no solution found
    
		if (DEBUG_CNTRTS) {
			printf("nbSolution = %d\n", nbSolutions);
		}
		//if there is a solution, put it into robot config
		for (i = 0; i < 7; i++) {
			if (i != 2) {
				j = i < 2 ? i : i - 1;
				q[i] = st_ikSolConfig[ct->num][0][j];
			}
		}
		q[2] = qm[0][2];
		if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
			return (FALSE);
		}
		return (TRUE);
	}
	return FALSE;
}

#if defined(USE_GBM)
/** Compute the inverse kinematic of Kuka LWR Arm. 
 * @param ct the constraint calling this function
 * @param iksol the solution of the ik (given by the planner)
 * @param qp the configuration pointer of the robot
 * @param dl
 * @return true if the function succeed false otherwise
 */
static int p3d_fct_lwr_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	int i = 0, j = 0, valid[8], ikChoice = p3d_get_ik_choice(),
  nbSolutions = 0;
	p3d_matrix4 r0Arm, armR0, armGrip, tmp;
	p3d_jnt * fixed;
	double q[7], qm[8][7], minmax[6][2];
  
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
	}
  
	fixed = ct->pasjnts[0]->rob->joints[ct->argu_i[0]]; //get the fixed joint
  
	//Position of the grip in the arm Base
	p3d_mat4Mult(ct->pasjnts[0]->prev_jnt->prev_jnt->abs_pos, ct->Tbase, r0Arm);
	p3d_matInvertXform(r0Arm, armR0);
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, tmp);
	p3d_mat4Mult(armR0, tmp, armGrip);
  
	if (DEBUG_CNTRTS) {
		p3d_mat4Print(armGrip, "armGrip");
		printf("fixed joint value = %f\n", fixed->dof_data[0].v);
	}
  
	if (iksol != -1) {
		ikChoice = IK_NORMAL;
	}
	if (ikChoice == IK_NORMAL || ikChoice == IK_UNIQUE) {
		if (ikChoice == IK_UNIQUE) {
			iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
		} else if (ikChoice == IK_NORMAL) {
			iksol = iksol != -1 ? iksol : ct->argu_i[2];
		}
		switch (ikLWRArmSolverUnique(fixed->dof_data[0].v, armGrip, q, iksol)) {		
      case 1: {
        if (DEBUG_CNTRTS)
          printf("IK LWR not valid\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 2: {
        if (DEBUG_CNTRTS)
          printf("IK LWR not valid, incorrect parameters\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 3: {
        if (DEBUG_CNTRTS)
          printf("Singularity\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }/*
        case -3: {
        if (DEBUG_CNTRTS)
				printf("Singularity joint 0\n");
        if (st_niksol)
				st_niksol[ct->num] = 0;
        return (FALSE);
        }
        case -4: {
        if (DEBUG_CNTRTS)
				printf("Singularity joint 3\n");
        if (st_niksol)
				st_niksol[ct->num] = 0;
        return (FALSE);
        }*/
      default: {
        if (DEBUG_CNTRTS) {
          printf("jnt");
          for (i = 0; i < 7; i++) {
            printf("q[%d] = %f\n", i, q[i]);
          }
        }
        if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
          if (DEBUG_CNTRTS) {
            //             for(i = 1; i < 9; i++){
            //               printf("test for sol = %d\n", i);
            //               if(ikKUKAArmSolverUnique(fixed->dof_data[0].v,aArray,alphaArray,dArray,thetaArray, armGrip,
            //                             q, ct->argu_i[1],i) == 1){
            //                 if(p3d_check_joints_bounds(q, ct, qp, dl)){
            //                   printf("Solution %d valide\n", i);
            //                 }else{
            //                   printf("Solution %d invalide\n", i);
            //                 }
            //               }
            //             }
          }
          return (FALSE);
        }
        if (st_niksol) {
          st_niksol[ct->num] = 1;
          st_iksol[ct->num][0] = iksol;
          for (i = 0; i < 7; i++) {
            if (i != 2) {//the fixed joint
              j = i < 2 ? i : i - 1;
              st_ikSolConfig[ct->num][0][j] = q[i];
            }
          }
        }
        return (TRUE);
      }
		}
	} else if (ikChoice == IK_MULTISOL) {
		ikLWRArmSolver(fixed->dof_data[0].v, armGrip, qm, valid);
		if (DEBUG_CNTRTS) {
			for (i = 1; i <= 8; i++) {
				printf("solution: %d, %d\n", i, ikLWRArmSolverUnique(
                                                             fixed->dof_data[0].v, armGrip, q, i));
			}
		}
		for (i = 0; i < 6; i++) {
			p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                             &minmax[i][0], &minmax[i][1]);
		}
		if ((nbSolutions = p3d_valid_solutions(ct, qm, valid, minmax, ct->num))
				== 0)
			return FALSE; //no solution found
    
		if (DEBUG_CNTRTS) {
			printf("nbSolution = %d\n", nbSolutions);
		}
		//if there is a solution, put it into robot config
		for (i = 0; i < 7; i++) {
			if (i != 2) {
				j = i < 2 ? i : i - 1;
				q[i] = st_ikSolConfig[ct->num][0][j];
			}
		}
		q[2] = qm[0][2];
		if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
			return (FALSE);
		}
		return (TRUE);
	}
	return FALSE;
}

/** Compute the inverse kinematic of Kuka PR2 Arm. 
 * @param ct the constraint calling this function
 * @param iksol the solution of the ik (given by the planner)
 * @param qp the configuration pointer of the robot
 * @param dl
 * @return true if the function succeed false otherwise
 */
static int p3d_fct_pr2_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
  //printf("p3d_fct_pr2_arm_ik\n");
	int i = 0, j = 0, valid[8], ikChoice = p3d_get_ik_choice(),
  nbSolutions = 0;
	p3d_matrix4 r0Arm, armR0, armGrip, tmp;
	p3d_jnt * fixed;
	double q[7], qm[8][7], minmax[6][2];
  
  // TODO : set from Joints Data
  // This is the left arm values
  double minDoFs[7];
  double maxDoFs[7];
  
  
  // Left Arm
//  minDoFs[0] = -41;
//  minDoFs[1] = -30;
//  minDoFs[2] = -45;
//  minDoFs[3] = -133;
//  minDoFs[4] = -45;
//  minDoFs[5] = -119;
//  minDoFs[6] = -180;
//  
//  maxDoFs[0] = 131;
//  maxDoFs[1] = 80;
//  maxDoFs[2] = 223;
//  maxDoFs[3] = 0;
//  maxDoFs[4] = 223;
//  maxDoFs[5] = 0;
//  maxDoFs[6] = 180;
  
  // Right Arm
//  minDoFs[0] = -131;
//  minDoFs[1] = -30;
//  minDoFs[2] = -223;
//  minDoFs[3] = -133;
//  minDoFs[4] = -223;
//  minDoFs[5] = -119;
//  minDoFs[6] = -180;
//  
//  maxDoFs[0] = 41;
//  maxDoFs[1] = 80;
//  maxDoFs[2] = 45;
//  maxDoFs[3] = 0;
//  maxDoFs[4] = 45;
//  maxDoFs[5] = 0;
//  maxDoFs[6] = 180;
//  
//  for(unsigned int i=0;i<7;i++)
//  {
//    minDoFs[i] *= M_PI/180;
//    maxDoFs[i] *= M_PI/180;
//  }
  
  
  for(int i = 0, j = 0; i < 7; i++){
    if(i == 2){
      p3d_get_robot_jnt_bounds(ct->argu_i[0], &minDoFs[i], &maxDoFs[i]);
    }else{
      p3d_get_robot_jnt_bounds(ct->pasjnts[j]->num, &minDoFs[i], &maxDoFs[i]);
      j++;
    }
  }
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
	}
  
	fixed = ct->pasjnts[0]->rob->joints[ct->argu_i[0]]; //get the fixed joint
  
	//Position of the grip in the arm Base
	p3d_mat4Mult(ct->pasjnts[0]->prev_jnt->abs_pos, ct->Tbase, r0Arm);
	p3d_matInvertXform(r0Arm, armR0);
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, tmp);
	p3d_mat4Mult(armR0, tmp, armGrip);
  
	if (DEBUG_CNTRTS) {
		p3d_mat4Print(armGrip, "armGrip");
		printf("fixed joint value = %f\n", fixed->dof_data[0].v);
	}
  
	if (iksol != -1) {
		//ikChoice = IK_NORMAL;
    ikChoice = IK_UNIQUE;
	}
	if (ikChoice == IK_NORMAL || ikChoice == IK_UNIQUE) {
		
    // Choose the IK solution class
    if (ikChoice == IK_UNIQUE) {
			iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
		} else if (ikChoice == IK_NORMAL) {
			iksol = iksol != -1 ? iksol : ct->argu_i[2];
		}
    
		switch (ikPr2ArmSolverUnique(fixed->dof_data[0].v, minDoFs, maxDoFs, armGrip, q, i)) 
    {		
      case 1: {
        if (DEBUG_CNTRTS)
          printf("IK PR2 not valid\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 2: {
        if (DEBUG_CNTRTS)
          printf("IK PR2 not valid, incorrect parameters\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 3: {
        if (DEBUG_CNTRTS)
          printf("Singularity\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      default: {
        if (DEBUG_CNTRTS) {
          printf("jnt");
          for (i = 0; i < 7; i++) {
            printf("q[%d] = %f\n", i, q[i]);
          }
        }
        if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
          if (DEBUG_CNTRTS) {
            //             for(i = 1; i < 9; i++){
            //               printf("test for sol = %d\n", i);
            //               if(ikKUKAArmSolverUnique(fixed->dof_data[0].v,aArray,alphaArray,dArray,thetaArray, armGrip,
            //                             q, ct->argu_i[1],i) == 1){
            //                 if(p3d_check_joints_bounds(q, ct, qp, dl)){
            //                   printf("Solution %d valide\n", i);
            //                 }else{
            //                   printf("Solution %d invalide\n", i);
            //                 }
            //               }
            //             }
          }
          printf("Incorrect DoFs bounds\n");
          return (FALSE);
        }
        if (st_niksol) {
          st_niksol[ct->num] = 1;
          st_iksol[ct->num][0] = iksol;
          for (i = 0; i < 7; i++) {
            if (i != 2) {//the fixed joint
              j = i < 2 ? i : i - 1;
              st_ikSolConfig[ct->num][0][j] = q[i];
            }
          }
        }
        return (TRUE);
      }
		}
	} else if (ikChoice == IK_MULTISOL) {
		ikPr2ArmSolver(fixed->dof_data[0].v, armGrip, qm, valid);
		if (DEBUG_CNTRTS) {
			for (i = 1; i <= 8; i++) {
				printf("solution: %d, %d\n", i, ikPr2ArmSolverUnique(
                                                             fixed->dof_data[0].v, minDoFs, maxDoFs, armGrip, q, i));
			}
		}
		for (i = 0; i < 6; i++) {
			p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                             &minmax[i][0], &minmax[i][1]);
		}
		if ((nbSolutions = p3d_valid_solutions(ct, qm, valid, minmax, ct->num))
				== 0)
			return FALSE; //no solution found
    
		if (DEBUG_CNTRTS) {
			printf("nbSolution = %d\n", nbSolutions);
		}
		//if there is a solution, put it into robot config
		for (i = 0; i < 7; i++) {
			if (i != 2) {
				j = i < 2 ? i : i - 1;
				q[i] = st_ikSolConfig[ct->num][0][j];
			}
		}
		q[2] = qm[0][2];
		if (!p3d_check_joints_bounds(q, ct, qp, dl)) {
			return (FALSE);
		}
		return (TRUE);
	}
	return FALSE;
}
#endif

/* -- functions for HUMAN ARM IK (of R7humanArm) -- */
static int p3d_set_R7_human_arm_ik(p3d_cntrt_management * cntrt_manager,
                                   int nb_pas, p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                   int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_jnt * jnt_arrPt[7];
	int jnt_dof_arrPt[7];
	int rob_dof_arrPt[7];
	int i;
	double dis1;
	double dis2;
  
	if (nb_pas == 7) {
		for (i = 0; i < 7; i++) {
			if (pas_jntPt[i]->type != P3D_ROTATE) {
				return FALSE;
			}
			jnt_arrPt[i] = pas_jntPt[i];
			jnt_dof_arrPt[i] = pas_jnt_dof[i];
			rob_dof_arrPt[i] = pas_rob_dof[i];
		}
	}
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_R7_HUMAN_ARM_NAME,
                                   7, jnt_arrPt, jnt_dof_arrPt, rob_dof_arrPt, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_R7_human_arm_ik;
		ct->ndval = 1;
    
		ct->col_pairs[0][0] = ct->actjnts[0]->o;
		ct->col_pairs[1][0] = ct->pasjnts[6]->o;
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = (int)Dval[0]; // swivel_angle
	ct->argu_d[1] = (int)Dval[1]; // left arm (0)/ right arm (1)
  
	ct->argu_i[0] = Ival[0]; // index of the base joint of the human
  
  
	// shoulder to elbow distance
	dis1 = sqrt(SQR(ct->pasjnts[3]->p0.x - ct->pasjnts[0]->p0.x)
              + SQR(ct->pasjnts[3]->p0.y - ct->pasjnts[0]->p0.y)
              + SQR(ct->pasjnts[3]->p0.z - ct->pasjnts[0]->p0.z));
  
	// elbow to wrist distance
	dis2 = sqrt(SQR(ct->pasjnts[4]->p0.x - ct->pasjnts[3]->p0.x)
              + SQR(ct->pasjnts[4]->p0.y - ct->pasjnts[3]->p0.y)
              + SQR(ct->pasjnts[4]->p0.z - ct->pasjnts[3]->p0.z));
  
	ct->argu_d[2] = dis1;
	ct->argu_d[3] = dis2;
	ct->argu_d[MAX_ARGU_CNTRT - 1] = dis1 + dis2;
	ct->argu_d[MAX_ARGU_CNTRT - 2] = fabs(dis1 - dis2);
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_R7_human_arm_ik(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl) {
	p3d_rob *r;
	double min, max;
	int i, j;
	p3d_matrix4 Tgrip, Tinv, Tdiff, TrotBase;
	double q[7], qlast[7];
  
	p3d_matrix4 Tejemp = { { 0, 0, 1, 0 }, { 0, 1, 0, 0 }, { -1, 0, 0, 0 }, {
    0, 0, 0, 1 } };
  
	p3d_matrix4 Tbase = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0,
    0, 0, 1 } };
  
	r = ct->pasjnts[0]->rob;
  
	p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, Tgrip);
  
	Tbase[0][3] = ct->pasjnts[2]->abs_pos[0][3];
	Tbase[1][3] = ct->pasjnts[2]->abs_pos[1][3];
	Tbase[2][3] = ct->pasjnts[2]->abs_pos[2][3];
	p3d_mat4Mult(Tbase, ct->Tatt2, TrotBase);
  
	p3d_matInvertXform(ct->pasjnts[0]->abs_pos_before_jnt, Tinv);
	p3d_mat4Mult(Tejemp, Tinv, Tdiff);
	/*
	 printf("==========================================\n");
	 p3d_mat4Print(Tdiff,"Tdiff");
	 p3d_mat4ExtractPosReverseOrder(Tdiff, &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]);
	 printf(" diff Rx = %10.6+f\n",q[3]*(180/M_PI));
	 printf(" diff Ry = %10.6+f\n",q[4]*(180/M_PI));
	 printf(" diff Rz = %10.6+f\n",180+(q[5]*(180/M_PI)));
	 */
  
	if (!compute_inverse_kinematics_R7_human_arm(q, Tgrip, TrotBase, Tdiff,
                                               ct->argu_d[0], ct->argu_d[2], ct->argu_d[3], (int)ct->argu_d[1])) {
		//return (FALSE);
    return (TRUE);
	} else {
		for (i = 0; i < ct->npasjnts; i++) {
			p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i], &min,
                             &max);
			qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
			if ((q[i] <= max) && (q[i] >= min)) {
				p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[i]);
			} else {
				for (j = 0; j < i; j++) {
					p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                          qlast[j]);
				}
			}
		}
	}
	return (TRUE);
}

#if defined(USE_GBM)
/** Compute the inverse kinematic of pa10_6 Arm. You can use a specific solution
 * of the ik given in the constraint declaration (the p3d file) or use multi solutions
 *
 * @param ct the constraint calling this function
 * @param iksol the solution of the ik (given by the planner)
 * @param qp the configuration pointer of the robot
 * @param dl
 * @return true if the function succeed false otherwise
 */
static int p3d_fct_pa10_6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp,
                                 double dl) {
	p3d_rob *r = NULL;
	double min = 0.0, max = 0.0;
	int i = 0, j = 0, k = 0, ikChoice = p3d_get_ik_choice(), valid[8];
	p3d_matrix4 Tobj, Tgrip, inv_jnt_mat;
	double qm[8][6], q[6], qlast[6];
  
	r = ct->pasjnts[0]->rob;
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
		/* necesario ???????? *//* solo si no es en generacion ??????? */
	}
  
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->abs_pos, inv_jnt_mat);
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, Tgrip);
	p3d_mat4Mult(inv_jnt_mat, Tgrip, Tobj);
  
	st_niksol[ct->num] = 0;
  
	if (iksol != -1) {
		ikChoice = IK_NORMAL;
	}
	if (ikChoice == IK_NORMAL || ikChoice == IK_UNIQUE) {
		if (ikChoice == IK_UNIQUE) {
			iksol = p3d_get_random_ikSol(ct->cntrt_manager, ct->num);
		} else if (ikChoice == IK_NORMAL) {
			iksol = iksol != -1 ? iksol : ct->argu_i[0];
		}
		switch (ikPA10ArmSolverUnique(Tobj, iksol, q)) {
      case 1: {
        if (DEBUG_CNTRTS)
          printf("IK pa10 not valid\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 2: {
        if (DEBUG_CNTRTS)
          printf("The Object is too far\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      case 3: {
        if (DEBUG_CNTRTS)
          printf("Singular configuration\n");
        if (st_niksol)
          st_niksol[ct->num] = 0;
        return (FALSE);
      }
      default: {
        for (i = 0; i < ct->npasjnts; i++) {
          p3d_jnt_get_dof_bounds(ct->pasjnts[i], ct->pas_jnt_dof[i],
                                 &min, &max);
          qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
          
          /* Test bounds of q */
          if (q[i] > max) {
            q[i]-= 2*M_PI;
            if ( (q[i] < min) || (q[i] > max)) {
              for (j = 0; j < i; j++) {
                p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                                qlast[j]);
              }
              return (FALSE);
            }
          }
          if (q[i] < min) {
            q[i]+= 2*M_PI;
            if ( (q[i] < min) || (q[i] > max)) {
              for (j = 0; j < i; j++) {
                p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j],
                                qlast[j]);
              }
              return (FALSE);
            }
          }
          p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[i]);
          
          //           if ((q[i] <= max) && (q[i] >= min)) {
          //             p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[i]);
          //           } else {
          //             for (j = 0; j < i; j++) {
          //               p3d_jnt_set_dof(ct->pasjnts[j], ct->pas_jnt_dof[j], qlast[j]);
          //             }
          //             return(FALSE);
          //           }
          
          
        }
        if (i == ct->npasjnts) {//if all joints bounds are ok
          st_niksol[ct->num] = 1;
          st_iksol[ct->num][0] = iksol;
          for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
            st_ikSolConfig[ct->num][0][j] = q[j];
          }
        }
        return (TRUE);
      }
		}
	} else if (ikChoice == IK_MULTISOL) {
    
		if (ikPA10ArmSolver(Tobj, valid, qm) == 0) {
			return FALSE; //There is no solutions
		} else {//verify joint bounds
      
			if (DEBUG_CNTRTS) {
				for (i = 1; i <= 8; i++) {
					printf("solution: %d, %d\n", i, ikPA10ArmSolverUnique(Tobj,
                                                                i, q));
				}
			}
			for (k = 0; k < 8; k++) {
				if (valid[k] == 0) {//it's a valid solution
					for (i = 0; i < ct->npasjnts; i++) {
						p3d_jnt_get_dof_bounds(ct->pasjnts[i],
                                   ct->pas_jnt_dof[i], &min, &max);
						qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i],
                                       ct->pas_jnt_dof[i]);
						if ((qm[k][i] <= max) && (qm[k][i] >= min)) {
							p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i],
                              qm[k][i]);
						} else {
							for (j = 0; j < i; j++) {
								p3d_jnt_set_dof(ct->pasjnts[j],
                                ct->pas_jnt_dof[j], qlast[j]);
							}
							break;
						}
					}
					if (i == ct->npasjnts) {//if all joints bounds are ok
						st_niksol[ct->num]
            = st_niksol[ct->num] >= 0 ? st_niksol[ct->num]
            + 1 : 1;//there is another valid solution
						st_iksol[ct->num][st_niksol[ct->num] - 1] = k + 1; //set the number of the solution
						for (j = 0; j < ct->npasjnts; j++) {//for each passive joint
							st_ikSolConfig[ct->num][st_niksol[ct->num] - 1][j]
              = qm[k][j];
						}
					}
				}
			}
		}
	}
	return (TRUE);
}

/* -- functions for IK (of pa10_6 ARM) -- */
static int p3d_set_pa10_6_arm_ik(p3d_cntrt_management * cntrt_manager,
                                 p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                 p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                                 double * dVal, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_matrix4 r0Base;
	int nb_act = 1, i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_PA10_6_ARM_IK_NAME,
                                   6, pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_pa10_6_arm_ik;
		ct->nival = 1;
		ct->ndval = 0;
		ct->nbSol = 8;//This constraint have a maximum of 8 solutions.
		//matrix of solutions
		/*
		 Sol 1 = 1 1 1
		 Sol 2 = 1 1 -1
		 Sol 3 = 1 -1 1
		 Sol 4 = 1 -1 -1
		 Sol 5 = -1 1 1
		 Sol 6 = -1 1 -1
		 Sol 7 = -1 -1 1
		 Sol 8 = -1 -1 -1
		 */
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_i[0] = iVal[0]; // Ik solution
  
	if (iVal[0] > ct->nbSol) { //if the user don't put a valid solution number
		return FALSE;
	}
  
	//Transformation between the platform and the arm
	p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->prev_jnt->pos0, r0Base); //inverse matrix R0->base
	p3d_mat4Mult(r0Base, ct->pasjnts[0]->prev_jnt->pos0, ct->Tbase); //store the transform matrix betweeen platform and the arm base
  
	for (i = 0; i < nb_act; i++) {
		if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {// if a active Dof Is a passiv Dof for another constraint enchain.
			p3d_enchain_cntrt(ct, act_rob_dof[i],
                        cntrt_manager->in_cntrt[act_rob_dof[i]]);
		} else {
			cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
		}
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
  
}

#endif

// int p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(
//                                                                   p3d_rob* robot, configPt q) {
// 	p3d_jnt *virObjJnt= NULL, *wristJnt= NULL;
// 	p3d_matrix4 TattInv, Twrist, TvirtObj;
// 	configPt q0= NULL;
// 	p3d_cntrt* cntrt_arm = NULL;
// 	int i=0;
//   
// 	for (i=0; i<=robot->njoints; i++) {
// 		if (robot->joints[i]->name==NULL)
// 			continue;
//     
// 		if (strcmp(robot->joints[i]->name, "virtual_object") == 0) {
// 			virObjJnt = robot->joints[i];
// 		}
// 	}
//   
// 	for (i=0; i<=robot->njoints; i++) {
// 		if (robot->joints[i]->name==NULL)
// 			continue;
//     
// 		if (strcmp(robot->joints[i]->name, "wristJoint") == 0) {
// 			wristJnt = robot->joints[i];
// 		}
// 	}
//   
// 	if (virObjJnt==NULL || wristJnt==NULL) {
// 		printf("FATAL_ERROR: the virtual object does not exist\n");
// 		return 0;
// 	}
// 	/* Look for the arm_IK constraint */
// 	for (i = 0; i < robot->cntrt_manager->ncntrts; i++) {
// 		cntrt_arm = robot->cntrt_manager->cntrts[i];
// 		if (strcmp(cntrt_arm->namecntrt, "p3d_pa10_6_arm_ik")==0) {
// 			break;
// 		}
// 	}
//   
// 	if (i == robot->cntrt_manager->ncntrts) {
// 		printf("FATAL_ERROR : arm_IK constraint does not exist\n");
// 		return 0;
// 	}
// 	if (cntrt_arm->active == 1) {
// 		return 1;
// 	}
// 	q0= p3d_alloc_config(robot);
// 	p3d_get_robot_config_into(robot, &q0);
//   
// 	p3d_set_and_update_this_robot_conf(robot, q);
//   
// 	p3d_mat4Copy(wristJnt->abs_pos, Twrist);
// 	p3d_matInvertXform(cntrt_arm->Tatt, TattInv);
// 	p3d_mat4Mult(Twrist, TattInv, TvirtObj);
//   
// 	p3d_mat4ExtractPosReverseOrder(TvirtObj, &q[virObjJnt->index_dof],
//                                  &q[virObjJnt->index_dof+1], &q[virObjJnt->index_dof+2],
//                                  &q[virObjJnt->index_dof+3], &q[virObjJnt->index_dof+4],
//                                  &q[virObjJnt->index_dof+5]);
//   
// 	p3d_activateCntrt(robot, cntrt_arm);
// 	p3d_set_and_update_this_robot_conf(robot, q);
//   
// 	p3d_desactivateCntrt(robot, cntrt_arm);
//   
// 	p3d_set_and_update_this_robot_conf(robot, q0);
// 	p3d_destroy_config(robot, q0);
// 	return 1;
// }

#ifdef LIGHT_PLANNER
/** \brief p3d_update_virtual_object_config_for_arm_ik_constraint
 * return 0 if there is an error
 */
int p3d_update_virtual_object_config_for_arm_ik_constraint( p3d_rob* robot, int armId, configPt q) {
	p3d_jnt *virObjJnt= NULL, *wristJnt= NULL;
	p3d_matrix4 TattInv, Twrist, TvirtObj;
	configPt q0= NULL;
	p3d_cntrt* cntrt_arm = NULL;
  int cntrtActive = false;

	if(robot->armManipulationData->size() == 0) {
		printf("%s: %d: p3d_update_virtual_object_config_for_arm_ik_constraint(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robot->name);
		return FALSE;
	}
	else {
    cntrt_arm = (*robot->armManipulationData)[armId].getCcCntrt();
    if (cntrt_arm == NULL) {
      printf("FATAL_ERROR : arm_IK constraint does not exist\n");
      return 0;
    }
    virObjJnt = cntrt_arm->actjnts[0];
    wristJnt = cntrt_arm->pasjnts[cntrt_arm->npasjnts-1];
	}

	if (virObjJnt == NULL || wristJnt == NULL) {
		printf("FATAL_ERROR: the virtual object does not exist or connot find end-eff Jnt\n");
		return 0;
	}


	if (cntrt_arm->active == 1) {
    cntrtActive = true;
		p3d_desactivateCntrt(robot, cntrt_arm);
	}
	q0 = p3d_get_robot_config(robot);

	p3d_set_and_update_this_robot_conf(robot, q);
  
	p3d_mat4Copy(wristJnt->abs_pos, Twrist);
	p3d_matInvertXform(cntrt_arm->Tatt, TattInv);
	p3d_mat4Mult(Twrist, TattInv, TvirtObj);

	p3d_mat4ExtractPosReverseOrder(TvirtObj, &q[virObjJnt->index_dof], &q[virObjJnt->index_dof+1], &q[virObjJnt->index_dof+2], &q[virObjJnt->index_dof+3], &q[virObjJnt->index_dof+4], &q[virObjJnt->index_dof+5]);

  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_activateCntrt(robot, cntrt_arm);
 // p3d_set_and_update_this_robot_conf(robot, q);
  p3d_get_robot_config_into(robot, &q);
  p3d_desactivateCntrt(robot, cntrt_arm);
  
  if(cntrtActive){
    p3d_activateCntrt(robot, cntrt_arm);
  }
	p3d_set_and_update_this_robot_conf(robot, q0);
	p3d_destroy_config(robot, q0);
	return 1;
}

/** \brief p3d_update_virtual_object_config
 * return 0 if there is an error
 */
int p3d_update_virtual_object_config(p3d_rob* robot, int armId, configPt q) {
	p3d_jnt *virObjJnt= NULL, *wristJnt= NULL;
	p3d_matrix4 TattInv, Twrist, TvirtObj;
	configPt q0= NULL;
	p3d_cntrt* cntrt_arm = NULL;
	int i=0;

	for (i=0; i<=robot->njoints; i++) {
		if (robot->joints[i]->name==NULL)
			continue;

		if (strcmp(robot->joints[i]->name, "virtual_object") == 0) {
			virObjJnt = robot->joints[i];
		}
	}

	for (i=0; i<=robot->njoints; i++) {
		if (robot->joints[i]->name==NULL)
			continue;

		if (strcmp(robot->joints[i]->name, "wristJoint") == 0) {
			wristJnt = robot->joints[i];
		}
	}

	if (virObjJnt==NULL || wristJnt==NULL) {
		printf("FATAL_ERROR: the virtual object does not exist\n");
		return 0;
	}

	if(robot->nbCcCntrts <=0) {
		printf("FATAL_ERROR : arm_IK constraint does not exist\n");
		return 0;
	}
	if(armId >robot->nbCcCntrts) {
		printf("FATAL_ERROR : wrong armId\n");
		return 0;
	}
        cntrt_arm = robot->ccCntrts[armId];

	if (cntrt_arm->active == 1) {
		return 1;
	}
	q0= p3d_alloc_config(robot);
	p3d_get_robot_config_into(robot, &q0);

	p3d_set_and_update_this_robot_conf(robot, q);

	p3d_mat4Copy(wristJnt->abs_pos, Twrist);
	p3d_matInvertXform(cntrt_arm->Tatt, TattInv);
	p3d_mat4Mult(Twrist, TattInv, TvirtObj);

	p3d_mat4ExtractPosReverseOrder(TvirtObj, &q[virObjJnt->index_dof],
                                 &q[virObjJnt->index_dof+1], &q[virObjJnt->index_dof+2],
                                 &q[virObjJnt->index_dof+3], &q[virObjJnt->index_dof+4],
                                 &q[virObjJnt->index_dof+5]);

	p3d_activateCntrt(robot, cntrt_arm);
	p3d_set_and_update_this_robot_conf(robot, q);

	p3d_desactivateCntrt(robot, cntrt_arm);

	p3d_set_and_update_this_robot_conf(robot, q0);
	p3d_destroy_config(robot, q0);
	return 1;
}
#endif

/**  
 * Computes Attached Matrix of a Inverse Kinematics constraint
 **/
void p3d_compute_Tatt(p3d_cntrt *ct) {
	p3d_matrix4 Twrist;
	p3d_mat4Copy(ct->pasjnts[ct->npasjnts-1]->abs_pos, Twrist);
	p3d_matrix4 TvirtObjInv;
	p3d_matInvertXform(ct->actjnts[0]->abs_pos, TvirtObjInv);
	p3d_mat4Mult(TvirtObjInv, Twrist, ct->Tatt);
	p3d_mat4Print(ct->Tatt, "Tatt");
}

#ifdef LIGHT_PLANNER
//! Updates the pose of the "virtual object" joint (active joint of an IK constraint)
//! from the current pose of the "end effector" joint (last passive joint of an IK constraint).
//! NB: the IK constraint is activated by the function.
int p3d_update_virtual_object_pose(p3d_rob* robotPt) {
	p3d_matrix4 T, Tatt_inv;
	p3d_jnt *endEffectorJnt= NULL;
  
	if(robotPt==NULL) {
		printf("%s: %d: p3d_update_virtual_object_pose(): input p3d_rob* is NULL.\n", __FILE__, __LINE__);
		return FALSE;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_update_virtual_object_pose(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
	else {
		endEffectorJnt= robotPt->ccCntrts[0]->pasjnts[robotPt->ccCntrts[0]->npasjnts-1];
	}
  
	p3d_matInvertXform(robotPt->ccCntrts[0]->Tatt, Tatt_inv);
	p3d_mat4Mult(endEffectorJnt->abs_pos, Tatt_inv, T);
	p3d_set_virtual_object_pose(robotPt, T);
  
	return TRUE;
}

int p3d_update_virtual_object_pose_in_config(p3d_rob* robotPt, configPt q) {
  
	if(robotPt==NULL) {
		printf("%s: %d: p3d_update_virtual_object_pose_in_config(): input p3d_rob* is NULL.\n", __FILE__, __LINE__);
		return FALSE;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_update_virtual_object_pose_in_config(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
  
	configPt q0= NULL;
  
	q0= p3d_alloc_config(robotPt);
	p3d_get_robot_config_into(robotPt, &q0);
  
	p3d_set_and_update_this_robot_conf(robotPt, q);
  
	p3d_update_virtual_object_pose(robotPt);
  
	p3d_get_robot_config_into(robotPt, &q);
  
	deactivateCcCntrts(robotPt, -1);
  
	p3d_set_and_update_this_robot_conf(robotPt, q0);
	p3d_destroy_config(robotPt, q0);
  
	return TRUE;
}

/**
 * Sets the virtual object (active joint that controls a ccCnrt) pose of the given robot and updates the robot configuration with the closed chain constraint.
 * NB: If the virtual object pose is not reachable, the robot is kept in its current configuration. 
 * @param robotPt 
 * @param T desired pose matrix of the virtual object
 * @return 0 in case of success, 1 otherwise
 */
int p3d_set_virtual_object_pose(p3d_rob *robotPt, p3d_matrix4 T)
{
	int result;
	double x, y, z, rx, ry, rz;
	p3d_jnt *virtObjJnt= NULL;
	configPt q0= NULL;
	configPt q= NULL;
  
	if(robotPt==NULL) {
		printf("%s: %d: p3d_set_virtual_object_pose(): input p3d_rob* is NULL.\n", __FILE__, __LINE__);
		return FALSE;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_set_virtual_object_pose(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
	else {
		virtObjJnt= robotPt->ccCntrts[0]->actjnts[0];
	}
  
	if(virtObjJnt==NULL) {
		printf("%s: %d: p3d_set_virtual_object_pose(): \"%s\"->ccCntrts[0]->actjnts[0] is NULL.\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
  
	q0= p3d_alloc_config(robotPt);
	p3d_get_robot_config_into(robotPt, &q0);
  
	p3d_mat4ExtractPosReverseOrder(T, &x, &y, &z, &rx, &ry, &rz);
  
	q= p3d_alloc_config(robotPt);
	p3d_get_robot_config_into(robotPt, &q);
  
	q[virtObjJnt->index_dof] = x;
	q[virtObjJnt->index_dof + 1] = y;
	q[virtObjJnt->index_dof + 2] = z;
	q[virtObjJnt->index_dof + 3] = rx;
	q[virtObjJnt->index_dof + 4] = ry;
	q[virtObjJnt->index_dof + 5] = rz;
  
	activateCcCntrts(robotPt, -1, true);
	result= p3d_set_and_update_this_robot_conf(robotPt, q);
  
	if(result==FALSE) {
		p3d_set_and_update_this_robot_conf(robotPt, q0);
	}
	else {
		p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_POS);
	}
  
	p3d_destroy_config(robotPt, q0);
	p3d_destroy_config(robotPt, q);
  
	return result;
}

/**
 * Sets the virtual object (active joint that controls a ccCnrt) pose of the given robot and updates the robot configuration with the closed chain constraint.
 * NB: if the virtual object pose is not reachable, the robot is kept in its current configuration. 
 * @param robotPt 
 * @param x coordinate along world X axis
 * @param y coordinate along world Y axis
 * @param z coordinate along world Z axis
 * @param rx Euler angle around world X axis (in radians)
 * @param ry Euler angle around world Y axis (in radians)
 * @param rz Euler angle around world Z axis (in radians)
 * @return TRUE in case of success, FALSE otherwise
 */
int p3d_set_virtual_object_pose2(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz)
{
	int result;
	configPt q0= NULL;
	configPt q= NULL;
	p3d_jnt *virtObjJnt= NULL;
  
	if(robotPt==NULL) {
		printf("%s: %d: p3d_set_virtual_object_pose2(): input p3d_rob* is NULL.\n", __FILE__, __LINE__);
		return FALSE;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_set_virtual_object_pose2(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
	else {
		virtObjJnt= robotPt->ccCntrts[0]->actjnts[0];
	}
  
	if(virtObjJnt==NULL) {
		printf("%s: %d: p3d_set_virtual_object_pose2(): \"%s\"->ccCntrts[0]->actjnts[0] is NULL.\n", __FILE__, __LINE__,robotPt->name);
		return FALSE;
	}
  
	q0= p3d_alloc_config(robotPt);
	p3d_get_robot_config_into(robotPt, &q0);
  
	q= p3d_alloc_config(robotPt);
	p3d_get_robot_config_into(robotPt, &q);
  
	q[virtObjJnt->index_dof] = x;
	q[virtObjJnt->index_dof + 1] = y;
	q[virtObjJnt->index_dof + 2] = z;
	q[virtObjJnt->index_dof + 3] = rx;
	q[virtObjJnt->index_dof + 4] = ry;
	q[virtObjJnt->index_dof + 5] = rz;
  
	activateCcCntrts(robotPt, -1, true);
	result= p3d_set_and_update_this_robot_conf(robotPt, q);
  
	if(result==FALSE) {
		p3d_set_and_update_this_robot_conf(robotPt, q0);
	}
	else {
		p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_POS);
	}
  
	p3d_destroy_config(robotPt, q0);
	p3d_destroy_config(robotPt, q);
  
	return result;
}

/**
 * Gets the current pose of the virtual object (active joint that controls a ccCnrt).
 * @param robotPt pointer to the robot
 * @param T where to copy the pose matrix
 * @return 0 in case of success, 1 otherwise
 */
int p3d_get_virtual_object_pose(p3d_rob *robotPt, p3d_matrix4 T)
{
	p3d_jnt *virtObjJnt= NULL;
  
	if(robotPt==NULL)
	{
		printf("%s: %d: p3d_get_virtual_object_pose(): robot is NULL.\n",__FILE__,__LINE__);
		return 1;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_get_virtual_object_pose(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return 1;
	}
	else {
		virtObjJnt= robotPt->ccCntrts[0]->actjnts[0];
	}
  
	if(virtObjJnt==NULL) {
		printf("%s: %d: p3d_get_virtual_object_pose(): \"%s\"->ccCntrts[0]->actjnts[0] is NULL.\n", __FILE__, __LINE__,robotPt->name);
		return 1;
	}
  
	p3d_mat4Copy(virtObjJnt->abs_pos, T);
  
	return 0;
}

/**
 * Gets the current pose of the virtual object (active joint that controls a ccCnrt).
 * @param robotPt pointer to the robot
 * @param x X position in world frame
 * @param y Y position in world frame
 * @param z Z position in world frame
 * @param rx X Euler angle (in radians)
 * @param ry Y Euler angle (in radians)
 * @param rz Z Euler angle (in radians)
 * @return 0 in case of success, 1 otherwise
 */
int p3d_get_virtual_object_pose2(p3d_rob *robotPt, double *x, double *y, double *z, double *rx, double *ry, double *rz)
{
	p3d_jnt *virtObjJnt= NULL;
  
	if(robotPt==NULL)
	{
		printf("%s: %d: p3d_get_virtual_object_pose2(): robot is NULL.\n",__FILE__,__LINE__);
		return 1;
	}
  
	if(robotPt->nbCcCntrts==0) {
		printf("%s: %d: p3d_get_virtual_object_pose2(): robot \"%s\" should have a ccCntrt (closed chained constraint).\n", __FILE__, __LINE__,robotPt->name);
		return 1;
	}
	else {
		virtObjJnt= robotPt->ccCntrts[0]->actjnts[0];
	}
  
	if(virtObjJnt==NULL) {
		printf("%s: %d: p3d_get_virtual_object_pose(): \"%s\"->ccCntrts[0]->actjnts[0] is NULL.\n", __FILE__, __LINE__,robotPt->name);
		return 1;
	}
  
	*x= p3d_jnt_get_dof(virtObjJnt, 0);
	*y= p3d_jnt_get_dof(virtObjJnt, 1);
	*z= p3d_jnt_get_dof(virtObjJnt, 2);
	*rx= p3d_jnt_get_dof(virtObjJnt, 3);
	*ry= p3d_jnt_get_dof(virtObjJnt, 4);
	*rz= p3d_jnt_get_dof(virtObjJnt, 5);
  
	return 0;
}
#endif

/****************************************************************************/
/* Functions for prismatic actuator inverse kinematic computation */

/*--------------------------------------------------------------------------*/
/*! \brief Add a new constraint to compute prismatic actuator IK
 *
 *  \note This constraint does not require a joint placed at the end-effector
 *        However, the model must be in a valid (closed) configuration
 *
 *  \param cntrt_managerPt: the constraints manager used
 *  \param pas_jntPt:       the array of passive joint
 *  \param pas_jnt_dof:     the array of dof joint indice
 *  \param pas_rob_dof:     the array of dof robot indice
 *  \param act_jntPt:       the array of active joint
 *  \param act_jnt_dof:     the array of dof joint indice
 *  \param act_rob_dof:     the array of dof robot indice
 *  \param Dval:            the position of the attachment point in ref. act_jnt[0]
 *  \param ct_num:          the constraint number, -1 when creates it
 *                          >= 0 for updating
 *  \param state:           TRUE when the constraint is activated
 *
 *  \return TRUE if the constraint has been successfully created
 *
 *  \internal
 */
static int p3d_set_prismatic_actuator_ik(
                                         p3d_cntrt_management * cntrt_managerPt, p3d_jnt **pas_jntPt,
                                         int *pas_jnt_dof, int *pas_rob_dof, p3d_jnt **act_jntPt,
                                         int *act_jnt_dof, int *act_rob_dof, double *Dval, int ct_num, int state) {
	p3d_cntrt *ctPt;
	p3d_matrix4 Tend, invTbase, Tbe;
	double theta_x, theta_y, dist_be;
	double vmin, vmax;
  
	/* Test the validity of passif joints */
	if ((pas_jntPt[0]->type != pas_jntPt[1]->type) || (pas_jntPt[0]->type
                                                     != P3D_KNEE) || (pas_jntPt[2]->type != P3D_TRANSLATE)) {
		PrintWarning(("Wrong type of passif joints !!!\n"));
		return FALSE;
	}
  
	if (ct_num < 0) {
		ctPt = p3d_create_generic_cntrts(cntrt_managerPt,
                                     CNTRT_PRISMATIC_ACTUATOR_NAME, 3, pas_jntPt, pas_jnt_dof,
                                     pas_rob_dof, 1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ctPt == NULL) {
			return FALSE;
		}
    
		ctPt->fct_cntrt = p3d_fct_prismatic_actuator_ik;
    
		/* initialize the parameters number */
		ctPt->ndval = 3;
		ctPt->nival = 0;
    
		ctPt->col_pairs[0][0] = ctPt->actjnts[0]->o;
		ctPt->col_pairs[1][0] = ctPt->pasjnts[2]->o;
    
		/* PROVISIONAL !!!!!!!!!!!!! */
		ctPt->Tatt[0][0] = 1.0;
		ctPt->Tatt[0][1] = 0.0;
		ctPt->Tatt[0][2] = 0.0;
		ctPt->Tatt[0][3] = Dval[0];
		ctPt->Tatt[1][0] = 0.0;
		ctPt->Tatt[1][1] = 1.0;
		ctPt->Tatt[1][2] = 0.0;
		ctPt->Tatt[1][3] = Dval[1];
		ctPt->Tatt[2][0] = 0.0;
		ctPt->Tatt[2][1] = 0.0;
		ctPt->Tatt[2][2] = 1.0;
		ctPt->Tatt[2][3] = Dval[2];
		ctPt->Tatt[3][0] = 0.0;
		ctPt->Tatt[3][1] = 0.0;
		ctPt->Tatt[3][2] = 0.0;
		ctPt->Tatt[3][3] = 1.0;
		/* !!!!!!!!!!!!!!!!!!!!!! */
    
		p3d_mat4Mult(ctPt->actjnts[0]->abs_pos, ctPt->Tatt, Tend);
		p3d_matInvertXform(ctPt->pasjnts[1]->pos0, invTbase);
		p3d_mat4Mult(invTbase, Tend, Tbe);
    
		theta_x = -atan2(Tbe[1][3], Tbe[2][3]);
		theta_y = atan2(Tbe[0][3], (Tbe[2][3] / cos(theta_x)));
		dist_be = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3]));
    
		ctPt->argu_d[0] = theta_x - p3d_jnt_get_dof_deg(ctPt->pasjnts[0],
                                                    ctPt->pas_jnt_dof[0]);
		ctPt->argu_d[1] = theta_y - p3d_jnt_get_dof_deg(ctPt->pasjnts[1],
                                                    ctPt->pas_jnt_dof[1]);
		ctPt->argu_d[2] = dist_be - p3d_jnt_get_dof_deg(ctPt->pasjnts[2],
                                                    ctPt->pas_jnt_dof[2]);
    
		/* max. extension */
		p3d_jnt_get_dof_bounds(ctPt->pasjnts[2], ctPt->pas_jnt_dof[2], &vmin,
                           &vmax);
		ctPt->argu_d[MAX_ARGU_CNTRT - 1] = ctPt->argu_d[2] + vmax;
		/* min. extension */
		ctPt->argu_d[MAX_ARGU_CNTRT - 2] = ctPt->argu_d[2];
    
	} else {
		ctPt = cntrt_managerPt->cntrts[ct_num];
	}
  
	if ((!state) || (!(ctPt->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_managerPt, ctPt, state)) {
			return FALSE;
		}
	}
	ctPt->active = state;
	last_cntrt_set = ctPt;
	return (TRUE);
}

/*--------------------------------------------------------------------------*/
/*! \brief Compute IK of prismatic actuator
 *
 *  \param ctPt: the constraints used
 *
 *  \return TRUE if the constraint has been successfully computed
 *
 *  \internal
 */
static int p3d_fct_prismatic_actuator_ik(p3d_cntrt *ctPt, int iksol,
                                         configPt qp, double dl) {
	double min, max;
	p3d_matrix4 Tend, invTbase, Tbe;
	double q[3], qlast[3];
	int i, j;
  
	//  if(p3d_go_into_cntrt_fct(ctPt)) {
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ctPt->pasjnts[0]->rob);
	}
  
	p3d_mat4Mult(ctPt->actjnts[0]->abs_pos, ctPt->Tatt, Tend);
	p3d_matInvertXform(ctPt->pasjnts[1]->pos0, invTbase);
	p3d_mat4Mult(invTbase, Tend, Tbe);
  
	q[0] = -atan2(Tbe[1][3], Tbe[2][3]) - ctPt->argu_d[0];
	q[1] = atan2(Tbe[0][3], (Tbe[2][3] / cos(q[0]))) - ctPt->argu_d[1];
	q[2] = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3]))
  - ctPt->argu_d[2];
  
	for (i = 0; i < ctPt->npasjnts; i++) {
		p3d_jnt_get_dof_bounds(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i], &min,
                           &max);
		qlast[i] = p3d_jnt_get_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i]);
		if ((q[i] <= max) && (q[i] >= min)) {
			p3d_jnt_set_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i], q[i]);
		} else {
			for (j = 0; j < i; j++) {
				p3d_jnt_set_dof(ctPt->pasjnts[j], ctPt->pas_jnt_dof[j],
                        qlast[j]);
			}
			return (FALSE);
		}
	}
	//  }
	return (TRUE);
}

/*************************/

/*--------------------------------------------------------------------------*/
/*! \brief Add a new constraint to compute prismatic actuator-II IK
 *
 *  \note This constraint requires a joint placed at the end-effector
 *        placement on the platform
 *        The min. length of the actuator is also required
 *
 *  \param cntrt_managerPt: the constraints manager used
 *  \param pas_jntPt:       the array of passive joint
 *  \param pas_jnt_dof:     the array of dof joint indice
 *  \param pas_rob_dof:     the array of dof robot indice
 *  \param act_jntPt:       the array of active joint
 *  \param act_jnt_dof:     the array of dof joint indice
 *  \param act_rob_dof:     the array of dof robot indice
 *  \param l0:              the min. length of the actuator
 *  \param ct_num:          the constraint number, -1 when creates it
 *                          >= 0 for updating
 *  \param state:           TRUE when the constraint is activated
 *
 *  \return TRUE if the constraint has been successfully created
 *
 *  \internal
 */
static int p3d_set_prismatic_actuator_II_ik(
                                            p3d_cntrt_management * cntrt_managerPt, p3d_jnt **pas_jntPt,
                                            int *pas_jnt_dof, int *pas_rob_dof, p3d_jnt **act_jntPt,
                                            int *act_jnt_dof, int *act_rob_dof, double l0, int ct_num, int state) {
	p3d_cntrt *ctPt;
	double vmin, vmax;
	p3d_matrix4 invT;
  
	/* Test the validity of passif joints */
	if ((pas_jntPt[0]->type != pas_jntPt[1]->type) || (pas_jntPt[0]->type
                                                     != P3D_KNEE) || (pas_jntPt[2]->type != P3D_TRANSLATE)) {
		PrintWarning(("Wrong type of passif joints !!!\n"));
		return FALSE;
	}
  
	if (ct_num < 0) {
		ctPt = p3d_create_generic_cntrts(cntrt_managerPt,
                                     CNTRT_PRISMATIC_ACTUATOR_II_NAME, 3, pas_jntPt, pas_jnt_dof,
                                     pas_rob_dof, 1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ctPt == NULL) {
			return FALSE;
		}
    
		ctPt->fct_cntrt = p3d_fct_prismatic_actuator_II_ik;
    
		/* initialize the parameters number */
		ctPt->ndval = 1;
		ctPt->nival = 0;
    
		ctPt->col_pairs[0][0] = ctPt->actjnts[0]->o;
		ctPt->col_pairs[1][0] = ctPt->pasjnts[2]->o;
    
		/* WARNING !!! :
		 preliminary version : considerations about the model makes
		 the computation of initial values are not necessary
		 */
    
		ctPt->argu_d[2] = l0;
    
		/* max. extension */
		p3d_jnt_get_dof_bounds(ctPt->pasjnts[2], ctPt->pas_jnt_dof[2], &vmin,
                           &vmax);
		ctPt->argu_d[MAX_ARGU_CNTRT - 1] = l0 + vmax;
		/* min. extension */
		ctPt->argu_d[MAX_ARGU_CNTRT - 2] = l0;
    
		/* transformation between the preceding joint and the base knee */
		p3d_matInvertXform(ctPt->pasjnts[0]->prev_jnt->pos0, invT);
		p3d_mat4Mult(invT, ctPt->pasjnts[0]->pos0, ctPt->Tbase);
    
	} else {
		ctPt = cntrt_managerPt->cntrts[ct_num];
	}
  
	if ((!state) || (!(ctPt->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_managerPt, ctPt, state)) {
			return FALSE;
		}
	}
	ctPt->active = state;
	last_cntrt_set = ctPt;
	return (TRUE);
}

/*--------------------------------------------------------------------------*/
/*! \brief Compute IK of prismatic actuator-II
 *
 *  \param ctPt: the constraints used
 *
 *  \return TRUE if the constraint has been successfully computed
 *
 *  \internal
 */
static int p3d_fct_prismatic_actuator_II_ik(p3d_cntrt *ctPt, int iksol,
                                            configPt qp, double dl) {
	double min, max;
	p3d_matrix4 invTbase, Tb, Tbe;
	double q[3], qlast[3];
	int i, j;
  
	//  if(p3d_go_into_cntrt_fct(ctPt)) {
  
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ctPt->pasjnts[0]->rob);
	}
  
	p3d_mat4Mult(ctPt->pasjnts[0]->prev_jnt->abs_pos, ctPt->Tbase, Tb);
	p3d_matInvertXform(Tb, invTbase);
	p3d_mat4Mult(invTbase, ctPt->actjnts[0]->abs_pos, Tbe);
  
	/* WARNING !!! :
	 the configuration of the attachment-joint is not computed
	 */
  
	q[0] = -atan2(Tbe[1][3], Tbe[2][3]);
	q[1] = atan2(Tbe[0][3], (Tbe[2][3] / cos(q[0])));
	q[2] = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3]))
  - ctPt->argu_d[2];
  
	for (i = 0; i < ctPt->npasjnts; i++) {
		p3d_jnt_get_dof_bounds(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i], &min,
                           &max);
		qlast[i] = p3d_jnt_get_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i]);
		if ((q[i] <= max) && (q[i] >= min)) {
			p3d_jnt_set_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i], q[i]);
		} else {
			for (j = 0; j < i; j++) {
				p3d_jnt_set_dof(ctPt->pasjnts[j], ctPt->pas_jnt_dof[j],
                        qlast[j]);
			}
			return (FALSE);
		}
	}
	//  }
	return (TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for CTBot IK  -- */
static int p3d_set_CTBot_ik(p3d_cntrt_management * cntrt_manager,
                            p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                            p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                            int ct_num, int state) {
	p3d_rob *robPt;
	p3d_cntrt *ctPt = NULL;
	//double vmin,vmax;
	//p3d_matrix4 invT;
  
	// validate joints (TO DO)
  
  
	if (ct_num < 0) {
		ctPt = p3d_create_generic_cntrts(cntrt_manager, CNTRT_CTBOT_NAME, 13,
                                     pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                     act_rob_dof);
    
		if (ctPt == NULL) {
			return FALSE;
		}
    
		ctPt->fct_cntrt = p3d_fct_CTBot_ik;
    
		// NOTE : geometric data are coded "in hard" in the CTBot_MGI
		//        they should be automatically calculated from the model
		//        and stocked in the ct
    
		/* initialize the parameters number */
		ctPt->ndval = 0;
		ctPt->nival = 0;
    
		// Collision pair deactivation (NECESSARY !!!???)
		//ctPt->col_pairs[0][0] = ctPt->actjnts[0]->o;
		//ctPt->col_pairs[1][0] = ctPt->pasjnts[2]->o;
    
		// reference for platform position (???)
		robPt = ctPt->pasjnts[0]->rob;
		p3d_mat4Copy(robPt->joints[0]->pos0, ctPt->Tbase);
    
	} else {
		ctPt = cntrt_manager->cntrts[ct_num];
	}
  
	if ((!state) || (!(ctPt->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ctPt, state)) {
			return FALSE;
		}
	}
	ctPt->active = state;
	last_cntrt_set = ctPt;
	return (TRUE);
}

static int p3d_fct_CTBot_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	p3d_rob *r;
	//double min,max;
	int i;
	p3d_matrix4 Tbase, invTbase, Tplatf, Ff;
	//double f[3], z[3];
	double q[13];
	//double qlast[13];
  
	r = ct->pasjnts[0]->rob;
  
	//if(p3d_go_into_cntrt_fct(ct)) {
	p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);
	//}
  
	p3d_mat4Copy(ct->actjnts[0]->abs_pos, Tplatf);
	p3d_mat4Copy(ct->Tbase, Tbase);
	p3d_matInvertXform(Tbase, invTbase);
	p3d_mat4Mult(invTbase, Tplatf, Ff);
  
	/*   f[0] = Tplatf[0][3]-Tbase[0][3]; */
	/*   f[1] = Tplatf[1][3]-Tbase[1][3]; */
	/*   f[2] = Tplatf[2][3]-Tbase[2][3]; */
  
	/*   z[0] = Tplatf[0][2]; */
	/*   z[1] = Tplatf[1][2]; */
	/*   z[2] = Tplatf[2][2]; */
  
	if (!CTBot_MGI(Ff, q)) {
		return (FALSE);
	} else {
		for (i = 0; i < ct->npasjnts; i++) {
			/*       p3d_jnt_get_dof_bounds(ct->pasjnts[i],ct->pas_jnt_dof[i], &min, &max); */
			/*       qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]); */
			/*       if((q[i] <= max)&&(q[i] >= min)) { */
			/*  p3d_jnt_set_dof(ct->pasjnts[i],ct->pas_jnt_dof[i], q[i]); */
			/*       } */
			/*       else { */
			/*  for(j=0; j<i; j++) { */
			/*    p3d_jnt_set_dof(ct->pasjnts[j],ct->pas_jnt_dof[j], qlast[j]); */
			/*  } */
			/*  return(FALSE); */
			/*       } */
			//// NO JOINT LIMITS !!! ///////
			p3d_jnt_set_dof(ct->pasjnts[i], ct->pas_jnt_dof[i], q[i]);
			////////////////////////////////
		}
	}
  
	return (TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for RLG test -- */
static int p3d_set_in_sphere(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double x,
                             double y, double z, int ct_num, int state) {
	p3d_cntrt *ct;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_IN_SPHERE_NAME, 0,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_in_sphere;
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = x;
	ct->argu_d[1] = y;
	ct->argu_d[2] = z;
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_in_sphere(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	p3d_jnt *JE;
	p3d_vector3 posi_jnt;
	p3d_vector3 v1, X1, X2;
	double distance;
  
	JE = ct->actjnts[0];
  
	if (!p3d_get_RLG())
		p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
  
	p3d_jnt_get_cur_vect_point(JE, posi_jnt);
	X1[0] = posi_jnt[0];
	X1[1] = posi_jnt[1];
	X1[2] = posi_jnt[2];
	X2[0] = ct->argu_d[0];
	X2[1] = ct->argu_d[1];
	X2[2] = ct->argu_d[2];
	p3d_vectSub(X2, X1, v1);
	distance = sqrt(sqr(v1[0]) + sqr(v1[1]) + sqr(v1[2]));
	/*  printf("ditance_fct: %f\n",distance); */
  
	if (distance > ct->argu_d[3]) {
		return (FALSE);
	}
	return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */

static int p3d_set_jnts_relpos_bound(p3d_cntrt_management * cntrt_manager,
                                     p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                     p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                     int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_rob *robPt;
  
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_JNTS_RELPOS_BOUND,
                                   0, pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_jnts_relpos_bound;
    
		// indices of the 2 jnts
		ct->argu_i[0] = Ival[0]; // joint of the 1st jnt
		ct->argu_i[1] = Ival[1]; // joint of the 2nd jnt
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0];
	ct->argu_d[1] = Dval[1];
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_jnts_relpos_bound(p3d_cntrt *ct, int iksol, configPt qp,
                                     double dl) {
	p3d_jnt *J1, *J2;
	p3d_vector3 posJ1, posJ2, pos_diff;
	double distance;
  
	// WARNING : this shold not be made : can give problems for multiple robots
	p3d_rob *robPt;
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	////////////
  
	if (!p3d_get_RLG())
		p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ???
  
	J1 = robPt->joints[ct->argu_i[0]];
	J2 = robPt->joints[ct->argu_i[1]];
  
	p3d_jnt_get_cur_vect_point(J1, posJ1);
	p3d_jnt_get_cur_vect_point(J2, posJ2);
  
	p3d_vectSub(posJ2, posJ1, pos_diff);
	distance = (double) p3d_vectNorm(pos_diff);
  
	//printf("ctnum : %d  , distance : %f\n",ct->num,distance);
  
	if ((distance < ct->argu_d[0]) || (distance > ct->argu_d[1])) {
		return (FALSE);
	}
  
	return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for cntrt fixing the distance range of 2 jnts relative position -- */
static int p3d_set_fix_jnts_relpos(p3d_cntrt_management * cntrt_manager,
                                   p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                   int *Ival, int ct_num, int state) {
	p3d_cntrt *ct = NULL;
  
	if (ct_num < 0) {
		if (pas_jntPt[0]->type != P3D_FREEFLYER) {
			return FALSE;
		}
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_FIX_JNTS_RELPOS, 1,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 1, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_fix_jnts_relpos;
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_fix_jnts_relpos(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl) {
	p3d_rob *robot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_matrix4 passiveJntTrans;
	double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  
	if (!p3d_get_RLG())
		p3d_update_this_robot_pos_without_cntrt_and_obj(robot); // necessary ???
  
	p3d_mat4Mult(ct->actjnts[0]->abs_pos, ct->Tatt, passiveJntTrans);
	p3d_mat4ExtractPosReverseOrder(passiveJntTrans, &x, &y, &z, &rx, &ry, &rz);
	p3d_jnt_set_dof(ct->pasjnts[0], 0, x - ct->pasjnts[0]->pos0[0][3]);
	p3d_jnt_set_dof(ct->pasjnts[0], 1, y - ct->pasjnts[0]->pos0[1][3]);
	p3d_jnt_set_dof(ct->pasjnts[0], 2, z - ct->pasjnts[0]->pos0[2][3]);
	p3d_jnt_set_dof(ct->pasjnts[0], 3, rx);
	p3d_jnt_set_dof(ct->pasjnts[0], 4, ry);
	p3d_jnt_set_dof(ct->pasjnts[0], 5, rz);
	return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for 6R_bio_ik -- */
static int p3d_set_6R_bio_ik(p3d_cntrt_management * cntrt_manager,
                             p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                             p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                             int state) {
	p3d_jnt *jnt_prevPt, *jntPt;
	p3d_cntrt *ct;
	p3d_jnt * jnt_arrPt[6];
	int jnt_dof_arrPt[6];
	int rob_dof_arrPt[6];
	int i;
  
	/* set passive joints */
	jntPt = jnt_prevPt = pas_jntPt[0];
	for (i = 0; i < 6; i++) {
		if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || (jntPt->type
                                                               != P3D_ROTATE)) {
			return FALSE;
		}
		jnt_arrPt[i] = jntPt;
		jnt_dof_arrPt[i] = 0;
		rob_dof_arrPt[i] = pas_rob_dof[0] + jntPt->index_dof
    - jnt_arrPt[0]->index_dof;
    
		jnt_prevPt = jntPt;
		if (jntPt->next_jnt == NULL)
			jntPt = NULL;
		else
			// WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
			jntPt = jntPt->next_jnt[0];
	}
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_6R_BIO_IK_NAME, 6,
                                   jnt_arrPt, jnt_dof_arrPt, rob_dof_arrPt, 1, act_jntPt,
                                   act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_6R_bio_ik;
		/* initialize the parameters number */
		ct->ndval = 0;
		ct->nival = 0;
    
		ct->col_pairs[0][0] = ct->actjnts[0]->o;
		ct->col_pairs[1][0] = ct->pasjnts[5]->o;
    
		bio_set_ik(ct);
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_6R_bio_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	p3d_jnt *JE, *J;
	double **sol_configs;
	double **valid_sol_configs;
	int i;
	double min, max;
	double qlast[6];
	int nsol, nvalidsol, ns, imds;
	int fail;
	double pqdist, minpqdist, aqdist;
	configPt qc, qcm;
	double dlfact;
	double ljnt;
	int naj;
  
	// local iksol allocation
	if (st_iksol == NULL)
		p3d_init_iksol(ct->cntrt_manager);
  
	JE = ct->actjnts[0];
  
	p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
  
	sol_configs = (double **)malloc(sizeof(double *) * 16);
	for (i = 0; i < 16; i++) {
		sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	/* compute all ik solutions */
	nsol = bio_compute_ik(ct, sol_configs);
	if (!nsol) {
		for (i = 0; i < 16; i++) {
			free(sol_configs[i]);
		}
		free(sol_configs);
		return (FALSE);
		/*     return(TRUE); */
	}
  
	for (i = 0; i < 6; i++) {
		qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
	}
  
	valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
	for (i = 0; i < nsol; i++) {
		valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	ns = 0;
	nvalidsol = 0;
	while (ns < nsol) {
		/*     printf("\n"); */
		fail = 0;
		for (i = 0; (i < 6) && (!fail); i++) {
			p3d_jnt_get_dof_bounds(ct->pasjnts[i], 0, &min, &max);
			if ((sol_configs[ns][i] <= max) && (sol_configs[ns][i] >= min)) {
				valid_sol_configs[nvalidsol][i] = sol_configs[ns][i];
				/*    printf(" %f ",sol_configs[ns][i]); */
				if (i == 5)
					nvalidsol++;
			} else {
				fail = 1;
			}
		}
		/*     printf("\n"); */
		ns++;
	}
  
	// set st_niksol
	st_niksol[ct->num] = nvalidsol;
  
	/* WARNING :
	 iksol is not an argument !!!
	 it is get from the vector (which has been modified)
	 */
	if (p3d_get_ik_choice() == IK_UNIQUE)
		iksol = st_iksol[ct->num][0];
  
	if (nvalidsol > 0) {
		if (qp == NULL) {
			if (iksol != -1) {
				/* if specified iksol */
				ns = iksol - 1;
			} else {
				/* choice a solution at random */
				ns = (int)floor(p3d_random(0.0, (double)nvalidsol - EPS6));
			}
			for (i = 0; i < 6; i++) {
				p3d_jnt_set_dof(ct->pasjnts[i], 0, valid_sol_configs[ns][i]);
			}
      
			// f modif for alkanes
			nvalidsol = ns + 1;
		} else {
			/* choice solution depending on qp */
			/* NOTE : the test of distance is made with the whole conf.
			 this could be a problem when the system has high n dofs
			 -> the test should be made only for the passive dofs !!!
			 */
			minpqdist = P3D_HUGE;
			qc = p3d_alloc_config(JE->rob);
			qcm = p3d_alloc_config(JE->rob);
			imds = 0;
			for (ns = 0; ns < nvalidsol; ns++) {
				for (i = 0; i < 6; i++) {
					p3d_jnt_set_dof(ct->pasjnts[i], 0, valid_sol_configs[ns][i]);
				}
				p3d_get_robot_config_into(JE->rob, &qc);
				if (DEBUG_CNTRTS) {
					// visualizar qp y q
					p3d_set_robot_config(JE->rob, qp);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
					p3d_set_robot_config(JE->rob, qc);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
				}
				ljnt = 0.0;
				for (i = 0; i < 6; i++) {
					ljnt
          += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc,
                                       qp));
					//printf("ljnt_iter = %f\n",ljnt);
				}
				pqdist = sqrt(ljnt);
				//printf("pqdist_i = %f\n",pqdist);
				/*  qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
				if (pqdist < minpqdist) {
					minpqdist = pqdist;
					imds = ns;
					p3d_copy_config_into(JE->rob, qc, &qcm);
				}
			}
			// STEP DIST EVALUATION
			// este opcion no funciona bien
			/*       wqdist = p3d_dist_q1_q2(JE->rob,qp,qcm); */
			/*       p3d_destroy_config(JE->rob,qc); */
			/*       p3d_destroy_config(JE->rob,qcm); */
			/*       aqdist = wqdist - minpqdist; */
			/*       dlfact = 2.0 * (ct->npasjnts / (JE->rob->njoints - ct->npasjnts)); */
			/*       printf("minpqdist = %f, wqdist = %f, aqdist = %f, dlfact= %f\n",minpqdist,wqdist,aqdist,dlfact); */
			/*       if(minpqdist < dlfact * aqdist) { */
			// otro metodo :
			// para un loop : pas_qdist / npasj < factor * act_qdist / nactj
			// WARNING : en el caso de que no haya RLG !!!
			if ((ct->rlgPt != NULL) && (ct->rlgPt->rlgchPt != NULL))
				J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
			else
				J = JE->rob->joints[1];
			ljnt = 0.0;
			naj = 0;
			while (J != ct->pasjnts[0]) {
				naj++;
				ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
				//printf("ljnt_iter = %f\n",ljnt);
				// WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
				J = J->next_jnt[0];
			}
			aqdist = sqrt(ljnt);
			//printf("aqdist = %f\n",aqdist);
			dlfact = 500.0; // ???!!!
			p3d_destroy_config(JE->rob, qc);
			p3d_destroy_config(JE->rob, qcm);
      
			// **************************************
			//IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
			//           y utilizarlo en este test
			// **************************************
			// NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
			//printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
			//if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ????
			if ((minpqdist) < (dlfact * aqdist)) {
				//printf("  -> OK\n");
				for (i = 0; i < 6; i++) {
					p3d_jnt_set_dof(ct->pasjnts[i], 0,
                          valid_sol_configs[imds][i]);
				}
				nvalidsol = imds + 1; // need to add 1 to avoid returning 0 !
			} else {
				//printf("  -> FAIL\n");
				nvalidsol = 0;
			}
		}
	}
  
	if (nvalidsol == 0) {
		for (i = 0; i < 6; i++) {
			p3d_jnt_set_dof(ct->pasjnts[i], 0, qlast[i]);
		}
	}
  
	for (i = 0; i < nsol; i++) {
		free(valid_sol_configs[i]);
	}
	free(valid_sol_configs);
  
	for (i = 0; i < 16; i++) {
		free(sol_configs[i]);
	}
	free(sol_configs);
  
	// local iksol
	st_iksol[ct->num][0] = nvalidsol;
  
	return (nvalidsol);
	/*   return (TRUE); */
}

//modif Mokhtar Not used
// static int p3d_fct_6R_bio_ik_alkanes(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
//   p3d_jnt *JE,*J;
//   double **sol_configs;
//   double **valid_sol_configs;
//   int i;
//   double min,max;
//   double qlast[6];
//   int nsol,nvalidsol,ns,imds;
//   int fail;
//   double pqdist,minpqdist,aqdist;
//   configPt qc,qcm;
//   double dlfact;
//   double ljnt;
//   int naj;
//   /////////////////////
//   int j;
//   double d1,d2,d3;
//   p3d_vector3 pos1,pos2,pos_diff;
//   ////////////////////
//   //static double mq1=1000,Mq1=-1000,mq2=1000,Mq2=-1000;
//
//
//   // local iksol allocation
//   if(st_iksol == NULL)
//     p3d_init_iksol(ct->cntrt_manager);
//
//   JE = ct->actjnts[0];
//
//   p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
//
//   sol_configs = (double **)malloc(sizeof(double *) * 16);
//   for(i = 0; i < 16; i++) {
//     sol_configs[i] = (double *)malloc(sizeof(double) * 6);
//   }
//
//   /* compute all ik solutions */
//   nsol = bio_compute_ik(ct,sol_configs);
//   if(!nsol) {
//     for(i = 0; i < 16; i++) {
//       free(sol_configs[i]);
//     }
//     free(sol_configs);
//     return(FALSE);
//     /*     return(TRUE); */
//   }
//
//   for(i=0; i<6; i++) {
//     qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
//   }
//
//   valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
//   for(i = 0; i < nsol; i++) {
//     valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
//   }
//
//   ns = 0;
//   nvalidsol = 0;
//   while(ns<nsol) {
//     /*     printf("\n"); */
//     fail = 0;
//     for(i=0; (i<6) && (!fail); i++) {
//       p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &min, &max);
//       if((sol_configs[ns][i] <= max)&&(sol_configs[ns][i] >= min)) {
//         valid_sol_configs[nvalidsol][i] = sol_configs[ns][i];
//         /*    printf(" %f ",sol_configs[ns][i]); */
//         if(i == 5)
//           nvalidsol++;
//       } else {
//         fail = 1;
//       }
//     }
//     /*     printf("\n"); */
//     ns++;
//   }
//
//   // set st_niksol
//   st_niksol[ct->num] = nvalidsol;
//
//   /* WARNING :
//      iksol is not an argument !!!
//      it is get from the vector (which has been modified)
//   */
//   if(look_iksol)
//     iksol = st_iksol[ct->num];
//
//   if(nvalidsol > 0) {
//     if(qp == NULL) {
//       if(iksol != -1) {
//         /* if specified iksol */
//         ns = iksol - 1;
//       }
//       // modif for alkanes
//       else {
//         // modif for alkanes
//         ////////////////////////
//         /* choice a solution at random */
//         /*  ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6)); */
//         /*       } */
//         /*       for(i=0; i<6 ; i++) { */
//         /*  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]); */
//         /*       }      */
//         ////////////////////////
//         ////////////////////////
//         for(j=0; j < nvalidsol;j++) {
//           for(i=0; i<6 ; i++) {
//             p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[j][i]);
//           }
//           p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
//
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[2],pos1);
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[6],pos2);
//           p3d_vectSub(pos1,pos2,pos_diff);
//           d1 = SQR((double) p3d_vectNorm(pos_diff));
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1);
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[6],pos2);
//           p3d_vectSub(pos1,pos2,pos_diff);
//           d2 = SQR((double) p3d_vectNorm(pos_diff));
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1);
//           p3d_jnt_get_cur_vect_point(JE->rob->joints[7],pos2);
//           p3d_vectSub(pos1,pos2,pos_diff);
//           d3 = SQR((double) p3d_vectNorm(pos_diff));
//
//
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[1],pos1); */
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos2); */
//           /*    p3d_vectSub(pos1,pos2,pos_diff); */
//           /*    d1 = SQR((double) p3d_vectNorm(pos_diff)); */
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[2],pos1); */
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[4],pos2); */
//           /*    p3d_vectSub(pos1,pos2,pos_diff); */
//           /*    d2 = SQR((double) p3d_vectNorm(pos_diff)); */
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1); */
//           /*    p3d_jnt_get_cur_vect_point(JE->rob->joints[5],pos2); */
//           /*    p3d_vectSub(pos1,pos2,pos_diff); */
//           /*    d3 = SQR((double) p3d_vectNorm(pos_diff)); */
//
//           printf("{ VECT 1 1 1 1 1 %f %f %f 255 0 0 1 }\n",d1,d2,d3);
//
//           // PLOT DE q1 y q2
//
//           //printf("{ VECT 1 1 1 1 1 %f %f %f 255 0 0 1 }\n",JE->rob->joints[1]->dof_data[0].v,1.0,0.0);
//           //printf("{ VECT 1 1 1 1 1 %f %f %f 0 255 0 1 }\n",JE->rob->joints[2]->dof_data[0].v,2.0,0.0);
//
//           /*    mq1 = 1.29; */
//           /*    if(fabs(JE->rob->joints[1]->dof_data[0].v) < mq1) { */
//           /*      mq1 = fabs(JE->rob->joints[1]->dof_data[0].v); */
//           /*      printf("minq1 = %f\n",mq1); */
//           /*    } */
//
//           /*    if(JE->rob->joints[1]->dof_data[0].v > Mq1) { */
//           /*      Mq1 = JE->rob->joints[1]->dof_data[0].v; */
//           /*      printf("maxq1 = %f\n",Mq1); */
//           /*    } */
//           /*    else if(JE->rob->joints[1]->dof_data[0].v < mq1) { */
//           /*      mq1 = JE->rob->joints[1]->dof_data[0].v; */
//           /*      printf("minq1 = %f\n",mq1); */
//           /*    } */
//           /*    if(JE->rob->joints[2]->dof_data[0].v > Mq2) { */
//           /*      Mq2 = JE->rob->joints[2]->dof_data[0].v; */
//           /*      printf("maxq2 = %f\n",Mq2); */
//           /*    } */
//           /*    else if(JE->rob->joints[2]->dof_data[0].v < mq2) { */
//           /*      mq2 = JE->rob->joints[2]->dof_data[0].v; */
//           /*      printf("minq2 = %f\n",mq2); */
//           /*    } */
//
//         }
//       }
//       ////////////////////////
//
//       // f modif for alkanes
//       nvalidsol = ns + 1;
//     } else {
//       /* choice solution depending on qp */
//       /* NOTE : the test of distance is made with the whole conf.
//          this could be a problem when the system has high n dofs
//       -> the test should be made only for the passive dofs !!!
//       */
//       minpqdist = P3D_HUGE;
//       qc = p3d_alloc_config(JE->rob);
//       qcm = p3d_alloc_config(JE->rob);
//       imds = 0;
//       for(ns=0; ns<nvalidsol; ns++) {
//         for(i=0; i<6; i++) {
//           p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]);
//         }
//         p3d_get_robot_config_into(JE->rob, &qc);
//         if(DEBUG_CNTRTS) {
//           // visualizar qp y q
//           p3d_set_robot_config(JE->rob,qp);
//           p3d_update_this_robot_pos_without_cntrt(JE->rob);
//           g3d_draw_allwin_active();
//           p3d_set_robot_config(JE->rob,qc);
//           p3d_update_this_robot_pos_without_cntrt(JE->rob);
//           g3d_draw_allwin_active();
//         }
//         ljnt = 0.0;
//         for(i=0; i<6; i++) {
//           ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc, qp));
//           //printf("ljnt_iter = %f\n",ljnt);
//         }
//         pqdist = sqrt(ljnt);
//         //printf("pqdist_i = %f\n",pqdist);
//         /*  qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
//         if(pqdist < minpqdist) {
//           minpqdist = pqdist;
//           imds = ns;
//           p3d_copy_config_into(JE->rob,qc,&qcm);
//         }
//       }
//       // STEP DIST EVALUATION
//       // este opcion no funciona bien
//       /*       wqdist = p3d_dist_q1_q2(JE->rob,qp,qcm); */
//       /*       p3d_destroy_config(JE->rob,qc); */
//       /*       p3d_destroy_config(JE->rob,qcm); */
//       /*       aqdist = wqdist - minpqdist; */
//       /*       dlfact = 2.0 * (ct->npasjnts / (JE->rob->njoints - ct->npasjnts)); */
//       /*       printf("minpqdist = %f, wqdist = %f, aqdist = %f, dlfact= %f\n",minpqdist,wqdist,aqdist,dlfact); */
//       /*       if(minpqdist < dlfact * aqdist) { */
//       // otro metodo :
//       // para un loop : pas_qdist / npasj < factor * act_qdist / nactj
//       // WARNING : en el caso de que no haya RLG !!!
//       if((ct->rlgPt != NULL) &&  (ct->rlgPt->rlgchPt != NULL))
//         J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
//       else
//         J = JE->rob->joints[1];
//       ljnt = 0.0;
//       naj = 0;
//       while(J != ct->pasjnts[0]) {
//         naj++;
//         ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
//         //printf("ljnt_iter = %f\n",ljnt);
//         // WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
//         J = J->next_jnt[0];
//       }
//       aqdist = sqrt(ljnt);
//       //printf("aqdist = %f\n",aqdist);
//       dlfact = 100.0;   // ???!!!
//       p3d_destroy_config(JE->rob,qc);
//       p3d_destroy_config(JE->rob,qcm);
//
//       // **************************************
//       //IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
//       //           y utilizarlo en este test
//       // **************************************
//       // NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
//       //printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
//       //if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ????
//       if((minpqdist) < (dlfact * aqdist)) {
//         //printf("  -> OK\n");
//         for(i=0; i<6; i++) {
//           p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i]);
//         }
//         nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
//       } else {
//         //printf("  -> FAIL\n");
//         nvalidsol = 0;
//       }
//     }
//   }
//
//   if(nvalidsol == 0) {
//     for(i=0; i<6; i++) {
//       p3d_jnt_set_dof(ct->pasjnts[i],0, qlast[i]);
//     }
//   }
//
//   for(i = 0; i < nsol; i++) {
//     free(valid_sol_configs[i]);
//   }
//   free(valid_sol_configs);
//
//   for(i = 0; i < 16; i++) {
//     free(sol_configs[i]);
//   }
//   free(sol_configs);
//
//   // local iksol
//   st_iksol[ct->num] = nvalidsol;
//
//   return (nvalidsol);
//   /*   return (TRUE); */
// }


/*****************************************************/

/* -- functions for 6R_bio_ik case of peptide bonds at 180 degrees -- */

/* WARNING : this function must be called with the peptide angles set at 180 */
static int p3d_set_6R_bio_ik_nopep(p3d_cntrt_management * cntrt_manager,
                                   p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                   p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int ct_num,
                                   int state) {
	p3d_jnt *jnt_prevPt, *jntPt;
	p3d_cntrt *ct;
	p3d_jnt * act_jnt_arrPt[6];
	int act_jnt_dof_arrPt[6];
	int act_rob_dof_arrPt[6];
	p3d_jnt * pas_jnt_arrPt[9];
	int pas_jnt_dof_arrPt[9];
	int pas_rob_dof_arrPt[9];
	int i;
	p3d_vector3 posJf, posJl, pos_diff;
  
	p3d_rob *r;
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
	/* set passive joints */
	jntPt = jnt_prevPt = pas_jntPt[0];
	for (i = 0; i < 9; i++) {
		if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || (jntPt->type
                                                               != P3D_ROTATE)) {
			return FALSE;
		}
		pas_jnt_arrPt[i] = jntPt;
		pas_jnt_dof_arrPt[i] = 0;
		pas_rob_dof_arrPt[i] = pas_rob_dof[0] + jntPt->index_dof
    - pas_jnt_arrPt[0]->index_dof;
    
		jnt_prevPt = jntPt;
		if (jntPt->next_jnt == NULL)
			jntPt = NULL;
		else
			jntPt = jntPt->next_jnt[jntPt->n_next_jnt - 1];
	}
  
	/*set active joints */
	if (act_jntPt[0]->type != P3D_FREEFLYER) {
		PrintError(("p3d_set_6R_bio_ik_nopep : act_jnt must be a P3D_FREEFLYER"));
		return FALSE;
	}
	for (i = 0; i < 6; i++) {
		act_jnt_arrPt[i] = act_jntPt[0];
		act_jnt_dof_arrPt[i] = i;
		act_rob_dof_arrPt[i] = act_jntPt[0]->index_dof + i;
		cntrt_manager->in_cntrt[act_rob_dof_arrPt[i]] = DOF_ACTIF;
	}
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager,
                                   CNTRT_6R_BIO_IK_NAME_NOPEP, 9, pas_jnt_arrPt,
                                   pas_jnt_dof_arrPt, pas_rob_dof_arrPt, 6, act_jnt_arrPt,
                                   act_jnt_dof_arrPt, act_rob_dof_arrPt);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_6R_bio_ik_nopep;
		/* initialize the parameters number */
		ct->ndval = 6;
		ct->nival = 1;
    
		/** variables used to keep end-frame situation **/
		/* flag to block end-frame (initially desactivated) */
		ct->argu_i[0] = 0;
		/* the end-frame (FREELYER) conf wil be stored in ct->argu_d[0,5] */
    
		ct->col_pairs[0][0] = ct->actjnts[0]->o;
		ct->col_pairs[1][0] = ct->pasjnts[8]->o;
    
		if (!bio_set_ik_nopep(ct)) {
			return FALSE;
		}
    
		// reshoot for this constraint (when there are several loops or several cntrts in the loop)
		if (cntrt_manager->ncntrts > 1) {
			ct->reshoot_ct = ct;
		}
		ct->nctud = 1;
		ct->ct_to_update = MY_ALLOC(pp3d_cntrt, 1);
		ct->ct_to_update[0] = ct;
    
		// max. extension
		// consider max. extension when all jnt are at 180
		// WARNING : consider the the chain is now set whit such a configuration
		p3d_jnt_get_cur_vect_point(ct->pasjnts[0], posJf);
		p3d_jnt_get_cur_vect_point(ct->pasjnts[8], posJl);
		p3d_vectSub(posJf, posJl, pos_diff);
		ct->argu_d[MAX_ARGU_CNTRT - 1] = (double) p3d_vectNorm(pos_diff);
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_6R_bio_ik_nopep(p3d_cntrt *ct, int iksol, configPt qp,
                                   double dl) {
	p3d_jnt *JE, *J;
	double **sol_configs;
	double **valid_sol_configs;
	int i, inci;
	double min, max;
	double qlast[9];
	int nsol, nvalidsol, ns, imds;
	int fail;
	double pqdist, minpqdist, aqdist;
	configPt qc, qcm;
	double dlfact;
	double ljnt;
	int naj;
  
	// local iksol allocation
	if (st_iksol == NULL)
		p3d_init_iksol(ct->cntrt_manager);
  
	/* the chain contains 3 AA
	 - the last joint (9) is fictive but required for refs
	 - joints 3, 6 and 9 are the peptide bonds -> fixded at 180 degrees
	 */
	p3d_jnt_get_dof_bounds(ct->pasjnts[2], 0, &min, &max);
	if (max < M_PI)
		p3d_jnt_set_dof(ct->pasjnts[2], 0, -M_PI);
	else
		p3d_jnt_set_dof(ct->pasjnts[2], 0, M_PI);
	p3d_jnt_get_dof_bounds(ct->pasjnts[5], 0, &min, &max);
	if (max < M_PI)
		p3d_jnt_set_dof(ct->pasjnts[5], 0, -M_PI);
	else
		p3d_jnt_set_dof(ct->pasjnts[5], 0, M_PI);
	p3d_jnt_get_dof_bounds(ct->pasjnts[8], 0, &min, &max);
	if (min == max) {
		p3d_jnt_set_dof(ct->pasjnts[8], 0, ct->pasjnts[8]->dof_data[0].v);
	} else {
		if (max < M_PI)
			p3d_jnt_set_dof(ct->pasjnts[8], 0, -M_PI);
		else
			p3d_jnt_set_dof(ct->pasjnts[8], 0, M_PI);
	}
  
	JE = ct->actjnts[0];
  
	p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
  
	// PRINT LOOP GEOMETRY
	// (only if the active jnt has been modified)
	if (bio_get_PRINT_LOOP_GEOM() && p3d_go_into_cntrt_fct(ct)
			&& (!p3d_jnt_get_dof_is_modified(JE->rob->joints[1], 0))) {
		bio_print_loop_geometry(ct);
	}
  
	sol_configs = (double **)malloc(sizeof(double *) * 16);
	for (i = 0; i < 16; i++) {
		sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	/* compute all ik solutions */
	nsol = bio_compute_ik_nopep(ct, sol_configs);
	if (!nsol) {
		for (i = 0; i < 16; i++) {
			free(sol_configs[i]);
		}
		free(sol_configs);
		return (FALSE);
		/*     return(TRUE); */
	}
  
	for (i = 0; i < 9; i++) {
		qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
	}
  
	valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
	for (i = 0; i < nsol; i++) {
		valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	ns = 0;
	nvalidsol = 0;
	while (ns < nsol) {
		/*     printf("\n"); */
		fail = 0;
		inci = 0;
		for (i = 0; (i < 8) && (!fail); i++) {
			if ((i == 2) || (i == 5)) {
				inci++;
			} else {
				p3d_jnt_get_dof_bounds(ct->pasjnts[i], 0, &min, &max);
				if ((sol_configs[ns][i-inci] <= max)
						&& (sol_configs[ns][i-inci] >= min)) {
					valid_sol_configs[nvalidsol][i-inci]
          = sol_configs[ns][i-inci];
					/*    printf(" %f ",sol_configs[ns][i-inci]); */
					if (i == 7)
						nvalidsol++;
				} else {
					fail = 1;
				}
			}
		}
		/*     printf("\n"); */
		ns++;
	}
  
	// set st_niksol
	st_niksol[ct->num] = nvalidsol;
  
	/* WARNING :
	 iksol is not an argument !!!
	 it is get from the vector (which has been modified)
	 */
	if (p3d_get_ik_choice() == IK_UNIQUE)
		iksol = st_iksol[ct->num][0];
  
	if (nvalidsol > 0) {
		if (qp == NULL) {
			if (iksol != -1) {
				/* if specified iksol */
				ns = iksol - 1;
			} else {
				/* choice a solution at random */
				ns = (int)floor(p3d_random(0.0, (double)nvalidsol - EPS6));
			}
			inci = 0;
			for (i = 0; i < 8; i++) {
				if ((i == 2) || (i == 5)) {
					inci++;
				} else {
					p3d_jnt_set_dof(ct->pasjnts[i], 0,
                          valid_sol_configs[ns][i-inci]);
				}
			}
			nvalidsol = ns + 1;
		} else {
			/* choice solution depending on qp */
			/* NOTE : the test of distance is made with the whole conf.
			 this could be a problem when the system has high n dofs
			 -> the test should be made only for the passive dofs !!!
			 */
			minpqdist = P3D_HUGE;
			qc = p3d_alloc_config(JE->rob);
			qcm = p3d_alloc_config(JE->rob);
			imds = 0;
			for (ns = 0; ns < nvalidsol; ns++) {
				inci = 0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						p3d_jnt_set_dof(ct->pasjnts[i], 0,
                            valid_sol_configs[ns][i-inci]);
					}
				}
				p3d_get_robot_config_into(JE->rob, &qc);
				if (DEBUG_CNTRTS) {
					// visualizar qp y q
					p3d_set_robot_config(JE->rob, qp);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
					p3d_set_robot_config(JE->rob, qc);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
				}
				ljnt = 0.0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0,
                                              qc, qp));
						//printf("ljnt_iter = %f\n",ljnt);
					}
				}
				pqdist = sqrt(ljnt);
				//printf("pqdist_i = %f\n",pqdist);
				/*  qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
				if (pqdist < minpqdist) {
					minpqdist = pqdist;
					imds = ns;
					p3d_copy_config_into(JE->rob, qc, &qcm);
				}
			}
			// STEP DIST EVALUATION
			// este opcion no funciona bien
			/*       wqdist = p3d_dist_q1_q2(JE->rob,qp,qcm); */
			/*       p3d_destroy_config(JE->rob,qc); */
			/*       p3d_destroy_config(JE->rob,qcm); */
			/*       aqdist = wqdist - minpqdist; */
			/*       dlfact = 2.0 * (ct->npasjnts / (JE->rob->njoints - ct->npasjnts)); */
			/*       printf("minpqdist = %f, wqdist = %f, aqdist = %f, dlfact= %f\n",minpqdist,wqdist,aqdist,dlfact); */
			/*       if(minpqdist < dlfact * aqdist) { */
			// otro metodo :
			// para un loop : pas_qdist / npasj < factor * act_qdist / nactj
			// WARNING : en el caso de que no haya RLG !!!
			if ((ct->rlgPt != NULL) && (ct->rlgPt->rlgchPt != NULL))
				J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
			else
				J = JE->rob->joints[1];
			ljnt = 0.0;
			naj = 0;
			while (J != ct->pasjnts[0]) {
				naj++;
				ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
				//printf("ljnt_iter = %f\n",ljnt);
				J = J->next_jnt[J->n_next_jnt - 1];
			}
			aqdist = sqrt(ljnt);
			//printf("aqdist = %f\n",aqdist);
			dlfact = 100.0; // ???!!!
			p3d_destroy_config(JE->rob, qc);
			p3d_destroy_config(JE->rob, qcm);
      
			// **************************************
			//IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
			//           y utilizarlo en este test
			// **************************************
			// NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
			//printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
			//if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ????
			//if((minpqdist) < (dlfact * aqdist)) {
			if ((minpqdist) < 10.0) {
				//printf("  -> OK\n");
				inci = 0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						p3d_jnt_set_dof(ct->pasjnts[i], 0,
                            valid_sol_configs[imds][i-inci]);
					}
				}
				nvalidsol = imds + 1; // need to add 1 to avoid returning 0 !
			} else {
				//printf("  -> FAIL\n");
				nvalidsol = 0;
			}
		}
	}
  
	if (nvalidsol == 0) {
		for (i = 0; i < 9; i++) {
			p3d_jnt_set_dof(ct->pasjnts[i], 0, qlast[i]);
		}
	}
  
	for (i = 0; i < nsol; i++) {
		free(valid_sol_configs[i]);
	}
	free(valid_sol_configs);
  
	for (i = 0; i < 16; i++) {
		free(sol_configs[i]);
	}
	free(sol_configs);
  
	// local iksol
	st_iksol[ct->num][0] = nvalidsol;
  
	return (nvalidsol);
	/*   return (TRUE); */
}

////////////////////////////////////////////////////////////////////////////////////

/* WARNING : this function must be called with the peptide angles set at 180 */
static int p3d_set_6R_bio_ik_nopep_new(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                       int ct_num, int state) {
	p3d_jnt *jnt_prevPt, *jntPt;
	p3d_cntrt *ct;
	//p3d_jnt * act_jnt_arrPt[6];
	//int act_jnt_dof_arrPt[6];
	//int act_rob_dof_arrPt[6];
	p3d_jnt * pas_jnt_arrPt[9];
	int pas_jnt_dof_arrPt[9];
	int pas_rob_dof_arrPt[9];
	int i;
	p3d_vector3 posJf, posJl, pos_diff;
  
	p3d_rob *r;
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
	/* set passive joints */
	jntPt = jnt_prevPt = pas_jntPt[0];
	for (i = 0; i < 9; i++) {
		if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || (jntPt->type
                                                               != P3D_ROTATE)) {
			return FALSE;
		}
		pas_jnt_arrPt[i] = jntPt;
		pas_jnt_dof_arrPt[i] = 0;
		pas_rob_dof_arrPt[i] = pas_rob_dof[0] + jntPt->index_dof
    - pas_jnt_arrPt[0]->index_dof;
    
		jnt_prevPt = jntPt;
		if (jntPt->next_jnt == NULL)
			jntPt = NULL;
		else
			jntPt = jntPt->next_jnt[jntPt->n_next_jnt - 1];
	}
  
	/*set active joints */
	/*   if(act_jntPt[0]->type != P3D_FREEFLYER) { */
	/*     PrintError(("p3d_set_6R_bio_ik_nopep_new : act_jnt must be a P3D_FREEFLYER")); */
	/*     return FALSE;  */
	/*   }  */
	/*   for(i=0; i<6; i++) { */
	/*     act_jnt_arrPt[i] = act_jntPt[0]; */
	/*     act_jnt_dof_arrPt[i] = i; */
	/*     act_rob_dof_arrPt[i] = act_jntPt[0]->index_dof + i; */
	/*     cntrt_manager->in_cntrt[act_rob_dof_arrPt[i]] = DOF_WITHOUT_CNTRT; */
	/*   } */
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager,
                                   CNTRT_6R_BIO_IK_NAME_NOPEP_NEW, 9, pas_jnt_arrPt,
                                   pas_jnt_dof_arrPt, pas_rob_dof_arrPt,
                                   //6, act_jnt_arrPt, act_jnt_dof_arrPt, act_rob_dof_arrPt);
                                   1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_6R_bio_ik_nopep_new;
		/* initialize the parameters number */
		ct->ndval = 12;
		ct->nival = 0;
    
		for (i = 0; i < 12; i++)
			ct->argu_d[i] = Dval[i];
    
		if (!bio_set_ik_nopep(ct)) {
			return FALSE;
		}
    
		// reshoot for this constraint (when there are several loops or several cntrts in the loop)
		if (cntrt_manager->ncntrts > 1) {
			ct->reshoot_ct = ct;
		}
		ct->nctud = 1;
		ct->ct_to_update = MY_ALLOC(pp3d_cntrt, 1);
		ct->ct_to_update[0] = ct;
    
		// max. extension
		// consider max. extension when all jnt are at 180
		// WARNING : consider the the chain is now set whit such a configuration
		p3d_jnt_get_cur_vect_point(ct->pasjnts[0], posJf);
		p3d_jnt_get_cur_vect_point(ct->pasjnts[8], posJl);
		p3d_vectSub(posJf, posJl, pos_diff);
		ct->argu_d[MAX_ARGU_CNTRT - 1] = (double) p3d_vectNorm(pos_diff);
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_6R_bio_ik_nopep_new(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl) {
	p3d_jnt *JE, *J;
	double **sol_configs;
	double **valid_sol_configs;
	int i, inci;
	double min, max;
	double qlast[9];
	int nsol, nvalidsol, ns, imds;
	int fail;
	double pqdist, minpqdist, aqdist;
	configPt qc, qcm;
	double dlfact;
	double ljnt;
	int naj;
  
	//if(p3d_go_into_cntrt_fct(ct)) {
  
	// local iksol allocation
	if (st_iksol == NULL)
		p3d_init_iksol(ct->cntrt_manager);
  
	/* the chain contains 3 AA
	 - the last joint (9) is fictive but required for refs
	 - joints 3, 6 and 9 are the peptide bonds -> fixded at 180 degrees
	 */
	p3d_jnt_get_dof_bounds(ct->pasjnts[2], 0, &min, &max);
	if (max < M_PI)
		p3d_jnt_set_dof(ct->pasjnts[2], 0, -M_PI);
	else
		p3d_jnt_set_dof(ct->pasjnts[2], 0, M_PI);
	p3d_jnt_get_dof_bounds(ct->pasjnts[5], 0, &min, &max);
	if (max < M_PI)
		p3d_jnt_set_dof(ct->pasjnts[5], 0, -M_PI);
	else
		p3d_jnt_set_dof(ct->pasjnts[5], 0, M_PI);
	p3d_jnt_get_dof_bounds(ct->pasjnts[8], 0, &min, &max);
	if (min == max) {
		p3d_jnt_set_dof(ct->pasjnts[8], 0, ct->pasjnts[8]->dof_data[0].v);
	} else {
		if (max < M_PI)
			p3d_jnt_set_dof(ct->pasjnts[8], 0, -M_PI);
		else
			p3d_jnt_set_dof(ct->pasjnts[8], 0, M_PI);
	}
  
	JE = ct->actjnts[0];
  
	p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
  
	sol_configs = (double **)malloc(sizeof(double *) * 16);
	for (i = 0; i < 16; i++) {
		sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	/* compute all ik solutions */
	nsol = bio_compute_ik_nopep_new(ct, sol_configs);
	if (!nsol) {
		for (i = 0; i < 16; i++) {
			free(sol_configs[i]);
		}
		free(sol_configs);
		return (FALSE);
		//return(TRUE);
	}
  
	for (i = 0; i < 9; i++) {
		qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
	}
  
	valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
	for (i = 0; i < nsol; i++) {
		valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
	}
  
	ns = 0;
	nvalidsol = 0;
	while (ns < nsol) {
		/*     printf("\n"); */
		fail = 0;
		inci = 0;
		for (i = 0; (i < 8) && (!fail); i++) {
			if ((i == 2) || (i == 5)) {
				inci++;
			} else {
				p3d_jnt_get_dof_bounds(ct->pasjnts[i], 0, &min, &max);
				// SETTING INTO MIN_MAX RANGE ------
				if ((max > M_PI) && (sol_configs[ns][i-inci] < min))
					sol_configs[ns][i-inci] += (2.0 * M_PI);
				if ((min < -M_PI) && (sol_configs[ns][i-inci] > max))
					sol_configs[ns][i-inci] -= (2.0 * M_PI);
				//----------------------------------
				if ((sol_configs[ns][i-inci] <= max)
						&& (sol_configs[ns][i-inci] >= min)) {
					valid_sol_configs[nvalidsol][i-inci]
          = sol_configs[ns][i-inci];
					/*    printf(" %f ",sol_configs[ns][i-inci]); */
					if (i == 7)
						nvalidsol++;
				} else {
					fail = 1;
				}
			}
		}
		/*     printf("\n"); */
		ns++;
	}
  
	if (nvalidsol == 0) {
		//printf("*** All IK solutions out of jnt bounds ***\n");
	}
  
	// set st_niksol
	st_niksol[ct->num] = nvalidsol;
  
	/* WARNING :
	 iksol is not an argument !!!
	 it is get from the vector (which has been modified)
	 */
	if (p3d_get_ik_choice() == IK_UNIQUE)
		iksol = st_iksol[ct->num][0];
  
	if (nvalidsol > 0) {
		if (qp == NULL) {
			if (iksol != -1) {
				/* if specified iksol */
				ns = iksol - 1;
			} else {
				/* choice a solution at random */
				//ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6));
				/* choice the closest configuration to qlast */
				minpqdist = P3D_HUGE;
				imds = 0;
				for (ns = 0; ns < nvalidsol; ns++) {
					inci = 0;
					ljnt = 0.0;
					for (i = 0; i < 8; i++) {
						if ((i == 2) || (i == 5)) {
							inci++;
						} else {
							ljnt += SQR(valid_sol_configs[ns][i-inci]
                          - qlast[i]);
							//printf("ljnt_iter = %f\n",ljnt);
						}
					}
					pqdist = sqrt(ljnt);
					//printf("pqdist_i = %f\n",pqdist);
					/*  qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
					if (pqdist < minpqdist) {
						minpqdist = pqdist;
						imds = ns;
					}
				}
        
				dlfact = 1.0; // ???!!!
        
				if ((minpqdist) < (dlfact)) {
					//printf("  -> OK\n");
					inci = 0;
					for (i = 0; i < 8; i++) {
						if ((i == 2) || (i == 5)) {
							inci++;
						} else {
							p3d_jnt_set_dof(ct->pasjnts[i], 0,
                              valid_sol_configs[imds][i-inci]);
						}
					}
					nvalidsol = imds + 1; // need to add 1 to avoid returning 0 !
				} else {
					//printf("  -> FAIL\n");
					nvalidsol = 0;
				}
			}
		} else {
			/* choice solution depending on qp */
			/* NOTE : the test of distance is made with the whole conf.
			 this could be a problem when the system has high n dofs
			 -> the test should be made only for the passive dofs !!!
			 */
			minpqdist = P3D_HUGE;
			qc = p3d_alloc_config(JE->rob);
			qcm = p3d_alloc_config(JE->rob);
			imds = 0;
			for (ns = 0; ns < nvalidsol; ns++) {
				inci = 0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						p3d_jnt_set_dof(ct->pasjnts[i], 0,
                            valid_sol_configs[ns][i-inci]);
					}
				}
				p3d_get_robot_config_into(JE->rob, &qc);
				if (DEBUG_CNTRTS) {
					// visualizar qp y q
					p3d_set_robot_config(JE->rob, qp);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
					p3d_set_robot_config(JE->rob, qc);
					p3d_update_this_robot_pos_without_cntrt(JE->rob);
					g3d_draw_allwin_active();
				}
				ljnt = 0.0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0,
                                              qc, qp));
						//printf("ljnt_iter = %f\n",ljnt);
					}
				}
				pqdist = sqrt(ljnt);
				//printf("pqdist_i = %f\n",pqdist);
				/*  qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
				if (pqdist < minpqdist) {
					minpqdist = pqdist;
					imds = ns;
					p3d_copy_config_into(JE->rob, qc, &qcm);
				}
			}
			// STEP DIST EVALUATION
			// esta opcion no funciona bien
			/*       wqdist = p3d_dist_q1_q2(JE->rob,qp,qcm); */
			/*       p3d_destroy_config(JE->rob,qc); */
			/*       p3d_destroy_config(JE->rob,qcm); */
			/*       aqdist = wqdist - minpqdist; */
			/*       dlfact = 2.0 * (ct->npasjnts / (JE->rob->njoints - ct->npasjnts)); */
			/*       printf("minpqdist = %f, wqdist = %f, aqdist = %f, dlfact= %f\n",minpqdist,wqdist,aqdist,dlfact); */
			/*       if(minpqdist < dlfact * aqdist) { */
			// otro metodo :
			// para un loop : pas_qdist / npasj < factor * act_qdist / nactj
			// WARNING : en el caso de que no haya RLG !!!
			if ((ct->rlgPt != NULL) && (ct->rlgPt->rlgchPt != NULL))
				J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
			else
				J = JE->rob->joints[1];
			ljnt = 0.0;
			naj = 0;
			while (J != ct->pasjnts[0]) {
				naj++;
				ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
				//printf("ljnt_iter = %f\n",ljnt);
				J = J->next_jnt[J->n_next_jnt - 1];
			}
			aqdist = sqrt(ljnt);
			//printf("aqdist = %f\n",aqdist);
			dlfact = 100.0; // ???!!!
			p3d_destroy_config(JE->rob, qc);
			p3d_destroy_config(JE->rob, qcm);
      
			// **************************************
			//IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
			//           y utilizarlo en este test
			// **************************************
			// NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
			//printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
			//if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ????
			//if((minpqdist) < (dlfact * aqdist)) {
			//printf("minpqdist = %f\n",minpqdist);
			if ((p3d_equal_config(JE->rob, qp, JE->rob->ROBOT_POS)) || // PROBLEMA CON ALGUNOS EJEMPLOS : LA PRIMERA CONFIGURACION REQUIERE UN "SALTO" MAYOR
					(minpqdist < 0.8)) { // REGLAJE DELICADO : DEBERIA DE DEPENDER DE LA LONGITUD DEL STEP !!! (Juan)
				inci = 0;
				for (i = 0; i < 8; i++) {
					if ((i == 2) || (i == 5)) {
						inci++;
					} else {
						p3d_jnt_set_dof(ct->pasjnts[i], 0,
                            valid_sol_configs[imds][i-inci]);
					}
				}
				nvalidsol = imds + 1; // need to add 1 to avoid returning 0 !
			} else {
				//printf("  -> FAIL\n");
				nvalidsol = 0;
			}
		}
	}
  
	if (nvalidsol == 0) {
		for (i = 0; i < 9; i++) {
			p3d_jnt_set_dof(ct->pasjnts[i], 0, qlast[i]);
		}
	}
  
	for (i = 0; i < nsol; i++) {
		free(valid_sol_configs[i]);
	}
	free(valid_sol_configs);
  
	for (i = 0; i < 16; i++) {
		free(sol_configs[i]);
	}
	free(sol_configs);
  
	// local iksol
	st_iksol[ct->num][0] = nvalidsol;
  
	return (nvalidsol);
}

//  else
//    return (TRUE);
//}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for BIO bkb Hbonds test -- */
static int p3d_set_bio_bkb_Hbond_cntrt(p3d_cntrt_management * cntrt_manager,
                                       p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                       p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                       int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_rob *robPt;
	p3d_matrix4 invT;
  
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_BIO_BKB_HBOND, 0,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_bio_bkb_Hbond_cntrt;
    
		// indices of first and last joints
		ct->argu_i[0] = Ival[0]; // first joint
		ct->argu_i[1] = Ival[1]; // last joint
		// index of the main constraint of the loop
		ct->argu_i[2] = Ival[2];
		// N-O bond (if 1) od O-N bond (if 0)
		ct->argu_i[3] = Ival[3];
    
		// compute Tatt and Tbase (dending on the kind of bond, N-O or O-N)
		// WARNING : suppose that the position of C is the joint position
		//           and that O is the second primitive in the object
		if (Ival[3] == 1) { // N-O bond
			p3d_mat4Copy(p3d_mat4IDENTITY, ct->Tbase);
			p3d_mat4Copy(robPt->joints[Ival[1]]->o->pol[1]->pos_rel_jnt,
                   ct->Tatt);
			// check if model contains H
			if (robPt->joints[Ival[0]]->o->np != 2) {
				printf("ERROR : p3d_set_bio_bkb_Hbond_cntrt : the model must contain Hs and be completely articulated\n");
				return FALSE;
			}
		} else { // O-N bond
			p3d_mat4Copy(robPt->joints[Ival[0]]->o->pol[1]->pos_rel_jnt,
                   ct->Tbase);
			p3d_matInvertXform(robPt->joints[Ival[1]]->abs_pos, invT);
			// NOTE : joint associanted with C has only one next_jnt
			p3d_matMultXform(invT,
                       robPt->joints[Ival[1]]->next_jnt[0]->abs_pos, ct->Tatt);
			// check if model contains H
			if (robPt->joints[Ival[1]]->o->np != 2) {
				printf("ERROR : p3d_set_bio_bkb_Hbond_cntrt : the model must contain Hs and be completely articulated\n");
				return FALSE;
			}
		}
    
		// enchain with main constraint of the loop
		p3d_add_to_cntrts_chain(cntrt_manager->cntrts[Ival[2]], ct,
                            cntrt_manager->cntrts[Ival[2]]->act_rob_dof[0]);
    
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0];
	ct->argu_d[1] = Dval[1];
	ct->argu_d[2] = Dval[2];
	ct->argu_d[3] = Dval[3];
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_bio_bkb_Hbond_cntrt(p3d_cntrt *ct, int iksol, configPt qp,
                                       double dl) {
	p3d_jnt *Jend, *Jbase, *JN;
	p3d_matrix4 Tbase, Tend, TH;
	p3d_vector3 posA1, posA2, posH, pos_diff;
	double d_A1A2, d_A1H, d_A2H;
	double angle;
  
	// WARNING : this shold not be made : can give problems for multiple robots
	p3d_rob *robPt;
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	////////////
  
	if (!p3d_get_RLG())
		p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ???
  
	Jbase = robPt->joints[ct->argu_i[0]];
	Jend = robPt->joints[ct->argu_i[1]];
  
	p3d_mat4Mult(Jbase->abs_pos, ct->Tbase, Tbase);
	p3d_mat4Mult(Jend->abs_pos, ct->Tatt, Tend);
  
	// H BOND LENGTH ////////////////////////////
  
	posA1[0] = Tbase[0][3];
	posA1[1] = Tbase[1][3];
	posA1[2] = Tbase[2][3];
	posA2[0] = Tend[0][3];
	posA2[1] = Tend[1][3];
	posA2[2] = Tend[2][3];
	p3d_vectSub(posA2, posA1, pos_diff);
	d_A1A2 = (double) p3d_vectNorm(pos_diff);
  
	//printf("ctnum : %d  , d_A1A2 : %f\n",ct->num,d_A1A2);
  
	if ((d_A1A2 < ct->argu_d[0]) || (d_A1A2 > ct->argu_d[1])) {
		return (FALSE);
	}
  
	// H BOND ANGLE ////////////////////////////
	if (ct->argu_i[3] == 1)
		JN = Jbase;
	else
		JN = Jend->next_jnt[0];
  
	// WARNING : suppose that H is the 2nd poly in the object attached ti JN !!!
	p3d_mat4Mult(JN->abs_pos, JN->o->pol[1]->pos_rel_jnt, TH);
	posH[0] = TH[0][3];
	posH[1] = TH[1][3];
	posH[2] = TH[2][3];
	p3d_vectSub(posH, posA1, pos_diff);
	d_A1H = (double) p3d_vectNorm(pos_diff);
	p3d_vectSub(posH, posA2, pos_diff);
	d_A2H = (double) p3d_vectNorm(pos_diff);
  
	angle = (180.0 / M_PI) * acos((sqr(d_A1H) + sqr(d_A2H) - sqr(d_A1A2)) / (2.0 * d_A1H * d_A2H));
  
	if ((angle < ct->argu_d[2]) || (angle > ct->argu_d[3])) {
		return (FALSE);
	}
  
	return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for BIO diS bonds test -- */
static int p3d_set_bio_diS_bond_cntrt(p3d_cntrt_management * cntrt_manager,
                                      p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                      p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, double *Dval,
                                      int *Ival, int ct_num, int state) {
	p3d_cntrt *ct;
	p3d_rob *robPt;
	p3d_poly *polPt = NULL;
  
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_BIO_DIS_BOND, 0,
                                   pas_jntPt, pas_jnt_dof, pas_rob_dof, 0, act_jntPt, act_jnt_dof,
                                   act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
		ct->fct_cntrt = p3d_fct_bio_diS_bond_cntrt;
    
		// indices of first and last joints
		ct->argu_i[0] = Ival[0]; // joint of the 1st S
		ct->argu_i[1] = Ival[1]; // joint of the 2nd S
    
		// compute Tatt and Tbase
		// WARNING : suppose that S is 1st (if no H) or 3rd (if H) primitive in the object
		if (robPt->joints[Ival[0]]->o->np == 1) {
			polPt = robPt->joints[Ival[0]]->o->pol[0];
		} else if (robPt->joints[Ival[0]]->o->np == 3) {
			polPt = robPt->joints[Ival[0]]->o->pol[2];
		} else {
			printf("ERROR : p3d_set_bio_diS_bond_cntrt : wrong np in obj %d\n",
             robPt->joints[Ival[0]]->o->num);
			return FALSE;
		}
		// Transf. fron jnt for 1st S is stored in Tbase
		p3d_mat4Copy(polPt->pos_rel_jnt, ct->Tbase);
    
		if (robPt->joints[Ival[1]]->o->np == 1) {
			polPt = robPt->joints[Ival[1]]->o->pol[0];
		} else if (robPt->joints[Ival[1]]->o->np == 3) {
			polPt = robPt->joints[Ival[1]]->o->pol[2];
		} else {
			printf("ERROR : p3d_set_bio_diS_bond_cntrt : wrong np in obj %d\n",
             robPt->joints[Ival[1]]->o->num);
			return FALSE;
		}
		// Transf. fron jnt for 2nd S is stored in Tatt
		p3d_mat4Copy(polPt->pos_rel_jnt, ct->Tatt);
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	ct->argu_d[0] = Dval[0];
	ct->argu_d[1] = Dval[1];
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

static int p3d_fct_bio_diS_bond_cntrt(p3d_cntrt *ct, int iksol, configPt qp,
                                      double dl) {
	p3d_jnt *Jend, *Jbase;
	p3d_matrix4 Tbase, Tend;
	p3d_vector3 posS1, posS2, posCB1, posCB2;
	p3d_vector3 CB1S1, S2S1, S2CB2;
	double distance, bondang;
  
	// WARNING : this shold not be made : can give problems for multiple robots
	p3d_rob *robPt;
	robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	////////////
  
	if (!p3d_get_RLG())
		p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ???
  
	Jbase = robPt->joints[ct->argu_i[0]];
	Jend = robPt->joints[ct->argu_i[1]];
  
	p3d_mat4Mult(Jbase->abs_pos, ct->Tbase, Tbase);
	p3d_mat4Mult(Jend->abs_pos, ct->Tatt, Tend);
  
	posS1[0] = Tbase[0][3];
	posS1[1] = Tbase[1][3];
	posS1[2] = Tbase[2][3];
	posS2[0] = Tend[0][3];
	posS2[1] = Tend[1][3];
	posS2[2] = Tend[2][3];
	p3d_vectSub(posS2, posS1, S2S1);
	distance = (double) p3d_vectNorm(S2S1);
  
	//printf("ctnum : %d  , distance : %f\n",ct->num,distance);
  
	// check bond length
  
	if ((distance < ct->argu_d[0]) || (distance > ct->argu_d[1])) {
		return (FALSE);
	}
  
	// check bond angles
	// NOTE: posCB1 =  Jbase->abs_pos ;  posCB2 =  Jend->abs_pos
	posCB1[0] = Jbase->abs_pos[0][3];
	posCB1[1] = Jbase->abs_pos[1][3];
	posCB1[2] = Jbase->abs_pos[2][3];
	posCB2[0] = Jend->abs_pos[0][3];
	posCB2[1] = Jend->abs_pos[1][3];
	posCB2[2] = Jend->abs_pos[2][3];
  
	p3d_vectSub(posCB1, posS1, CB1S1);
	p3d_vectSub(posS2, posCB2, S2CB2);
  
	bondang = (180.0 / M_PI) * acos(p3d_vectDotProd(S2S1, CB1S1)
                                  / (p3d_vectNorm(S2S1) * p3d_vectNorm(CB1S1)));
	if ((bondang < 100.0) || (bondang > 120.0)) {
		return (FALSE);
	}
	//printf("ctnum : %d  , bond_ang1 : %f\n",ct->num,bondang);
	bondang = (180.0 / M_PI) * acos(p3d_vectDotProd(S2S1, S2CB2)
                                  / (p3d_vectNorm(S2S1) * p3d_vectNorm(S2CB2)));
	if ((bondang < 100.0) || (bondang > 120.0)) {
		return (FALSE);
	}
	//printf("ctnum : %d  , bond_ang2 : %f\n",ct->num,bondang);
  
  
	return TRUE;
}

/*first active joint = the joint to change, second one gives the values for first joint bounds*/
static int p3d_fct_min_max_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl) {
	double startValue, finalMinValue, finalMaxValue;
	int i, I_can;
	double lastMinValue, lastMaxValue, lastDofValue;
  
	//   if (p3d_go_into_cntrt_fct(ct)) {//Search if the constraints depend on the modified joint
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);
	}
	startValue = p3d_jnt_get_dof_deg(ct->actjnts[1], ct->act_jnt_dof[1]);//Get the start value of a dof for the active joint
	finalMinValue = startValue < 0 ? ct->argu_d[0] - startValue : ct->argu_d[0];//compute the final value Min value of passive joint
	finalMaxValue = startValue < 0 ? ct->argu_d[1] : ct->argu_d[1] - startValue;//compute the final value Max value of passive joint
	p3d_jnt_get_dof_bounds_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                             &lastMinValue, &lastMaxValue);//save last dof bouds
	p3d_jnt_set_dof_bounds_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                             finalMinValue, finalMaxValue);//set min max bouds of the dof first active joint
	//check if the value of the dof is on the bounds
	lastDofValue = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
	if (lastDofValue < finalMinValue) {
		p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0], finalMinValue);
	}
	if (lastDofValue > finalMaxValue) {
		p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0], finalMaxValue);
	}
	if (DEBUG_CNTRTS) {
		printf("jnt2 = %f\t%f\n", startValue, (startValue / 180)*M_PI);
		printf("jnt3 = %f\t%f\n", lastDofValue, (lastDofValue / 180)*M_PI);
	}
	if (st_niksol) {
		st_niksol[ct->num] = 1;
		st_iksol[ct->num][0] = 1;
	}
	if (ct->enchained != NULL) {
		I_can = 1;
		for (i = 0; I_can && (i < ct->nenchained); i++) {
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 1);
			if (ct->enchained[i]->active)// multiple iksol is not compatible (yet) with enchained !
				I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i], -1,
                                               qp, dl);
			p3d_change_act_rob_dof_state(ct->enchained[i],
                                   ct->enchained_rob_dof[i], 0);
		}
		if (!I_can) {
			p3d_jnt_set_dof_bounds_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                                 lastMinValue, lastMaxValue);
			p3d_jnt_set_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0],
                          lastDofValue);
			return (FALSE);
		}
	}
	//   }
	return (TRUE);
}

static int p3d_fct_head_object_track(p3d_cntrt *ct, int iksol, configPt qp,
                                     double dl) {
	double min = 0.0, max = 0.0, angle = 0.0;
	p3d_vector3 objectPos, projVect;
	p3d_matrix4 invHead, ref;
	if (!TEST_PHASE) {
		p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
		/* necesario ???????? *//* solo si no es en generacion ??????? */
	}
	//compute the vector between the tilt jnt center and the object center
	p3d_jnt_get_cur_vect_point(ct->actjnts[0], objectPos);
  
	//get the angle between tilt x axis and the vector projection on x z plan (tilt degree)
	p3d_mat4Copy(ct->pasjnts[0]->prev_jnt->abs_pos, ref);
	for (int i = 0; i < 3; i++) {
		ref[i][3] = ct->pasjnts[0]->abs_pos[i][3];
	}
	p3d_matInvertXform(ref, invHead);
	p3d_xformPoint(invHead, objectPos, projVect);
  
	angle = atan2(projVect[1], projVect[0]);
	p3d_jnt_get_dof_bounds(ct->pasjnts[0], 0, &min, &max);
	if (angle > max) {
		p3d_jnt_set_dof(ct->pasjnts[0], 0, max);
	} else if (angle < min) {
		p3d_jnt_set_dof(ct->pasjnts[0], 0, min);
	} else {
		p3d_jnt_set_dof(ct->pasjnts[0], 0, angle);
	}
	if (st_niksol) {
		st_iksol[ct->num][0] = 1;
		st_niksol[ct->num] = 1;
		st_ikSolConfig[ct->num][0][0] = angle;
	}
	//get the angle between tilt x axis and the vector projection on x y plan (pan degree)
	p3d_mat4Copy(ct->pasjnts[0]->prev_jnt->abs_pos, ref);
	for (int i = 0; i < 3; i++) {
		ref[i][3] = ct->pasjnts[1]->abs_pos[i][3];
	}
	p3d_matInvertXform(ref, invHead);
	p3d_xformPoint(invHead, objectPos, projVect);
  
	angle = atan2(projVect[2], projVect[0]);
	p3d_jnt_get_dof_bounds(ct->pasjnts[1], 0, &min, &max);
	if (-angle > max) {
		p3d_jnt_set_dof(ct->pasjnts[1], 0, max);
	} else if (-angle < min) {
		p3d_jnt_set_dof(ct->pasjnts[1], 0, min);
	} else {
		p3d_jnt_set_dof(ct->pasjnts[1], 0, -angle);
	}
	if (st_niksol) {
		st_ikSolConfig[ct->num][0][1] = -angle;
	}
	return TRUE;
}

static int p3d_set_head_object_track(p3d_cntrt_management * cntrt_manager,
                                     p3d_jnt **pas_jntPt, int *pas_jnt_dof, int *pas_rob_dof,
                                     p3d_jnt **act_jntPt, int *act_jnt_dof, int *act_rob_dof, int *iVal,
                                     double * dVal, int ct_num, int state) {
	p3d_cntrt *ct;
	int nb_act = 1, i;
  
	if (ct_num < 0) {
		ct = p3d_create_generic_cntrts(cntrt_manager,
                                   CNTRT_HEAD_OBJECT_TRACK_NAME, 2, pas_jntPt, pas_jnt_dof,
                                   pas_rob_dof, 1, act_jntPt, act_jnt_dof, act_rob_dof);
		if (ct == NULL) {
			return FALSE;
		}
    
		ct->fct_cntrt = p3d_fct_head_object_track;
		ct->nival = 0;
		ct->ndval = 0;
		ct->nbSol = 1;//This constraint have a maximum of 1 solution1.
	} else {
		ct = cntrt_manager->cntrts[ct_num];
	}
  
	for (i = 0; i < nb_act; i++) {
		if (cntrt_manager->in_cntrt[act_rob_dof[i]] == DOF_PASSIF) {// if a active Dof Is a passiv Dof for another constraint enchain.
			p3d_enchain_cntrt(ct, act_rob_dof[i],
                        cntrt_manager->in_cntrt[act_rob_dof[i]]);
		} else {
			cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF;
		}
	}
  
	if ((!state) || (!(ct->active) && state)) {
		if (!p3d_update_jnts_state(cntrt_manager, ct, state)) {
			return FALSE;
		}
	}
	ct->active = state;
	last_cntrt_set = ct;
	return (TRUE);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/* function returning if there are active cntrts or not */

int p3d_actived_cntrts(p3d_cntrt_management *cntrt_manager) {
	p3d_cntrt *ct;
  
	if ((cntrt_manager == NULL) || (cntrt_manager->cntrts == NULL)) {
		return FALSE;
	} else {
		for (dbl_list_goto_first(cntrt_manager->cntrt_call_list); dbl_list_more(cntrt_manager->cntrt_call_list); dbl_list_next(cntrt_manager->cntrt_call_list)) {
			ct = (p3d_cntrt *) DBL_LIST_DATA(void *, cntrt_manager->cntrt_call_list);
			if (ct->active)
				return TRUE;
		}
	}
  
	return FALSE;
}

/*---------------------------------------------------------------------------*/

/****************************************************/
/****************************************************/
/*   FUNCTIONS TESTING CONSTRAINTS ON LOCAL PATHS   */
/****************************************************/
/****************************************************/

int p3d_check_cntrts_at_conf(p3d_rob *robotPt, configPt q) {
	p3d_cntrt *ct;
  
	p3d_set_robot_config(robotPt, q);
	p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
  
	/* change la configuration s'il y a des contraintes cinematiques */
	if (robotPt->cntrt_manager->cntrts != NULL) {
		for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list); dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
			ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
			if (ct->active)
				if (!(*ct->fct_cntrt)(ct, -1, NULL, 0.0))
					return FALSE;
		}
	}
  
	return TRUE;
}

int p3d_check_cntrts_at_conf_multisol(p3d_rob *robotPt, configPt q,
                                      configPt qp, double dl) {
	p3d_cntrt *ct;
  
	p3d_set_robot_config(robotPt, q);
	p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
  
	/* change la configuration s'il y a des contraintes cinematiques */
	if (robotPt->cntrt_manager->cntrts != NULL) {
		for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list); dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
			ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
			if (ct->active)
				if (!(*ct->fct_cntrt)(ct, -1, qp, dl))
					return FALSE;
		}
	}
  
	return TRUE;
}

/*---------------------------------------------------------------------------*/

int p3d_cntrt_localpath_classic_test(p3d_rob *robotPt,
                                     p3d_localpath *localpathPt, double *Kpath) {
	double u, du, umax;
	double dmax;
	int valid = 1;
	int end_localpath = 0;
	double *distances;
	int j;
	int njnt = robotPt->njoints;
	configPt qsave;
  
	if (localpathPt == NULL) {
		return FALSE;
	}
  
	/* Some curves can be decided unvalid by the user */
	if (localpathPt->valid == FALSE) {
		return FALSE;
	}
  
	if (p3d_actived_cntrts(robotPt->cntrt_manager)) {
		*Kpath = 0.0;
		umax = localpathPt->range_param;
		distances = MY_ALLOC(double, njnt + 1);
		/* NOTE :
		 we have chosen same discretization step than in collision checking
		 however it can be different !!!
		 */
		p3d_col_get_dmax(&dmax);
		dmax *= 10;
    
		/* current position of robot is saved */
		qsave = p3d_get_robot_config(robotPt);
    
		for (j = 0; j <= njnt; j++) {
			distances[j] = dmax;
		}
		u = 0.0;
		du = localpathPt->stay_within_dist(robotPt, localpathPt, u, FORWARD,
                                       distances);
    
		u = du;
		if (u > umax - EPS6) {
			end_localpath = 1;
		}
    
		while (!end_localpath) {
			/* position of the robot corresponding to parameter u */
			if (change_position_robot_without_obj(robotPt, localpathPt, u)) {
				p3d_set_and_update_this_robot_conf(robotPt, qsave);
				p3d_destroy_config(robotPt, qsave);
				MY_FREE(distances, double, njnt + 1);
				return FALSE;
			}
      
			*Kpath = u / localpathPt->range_param;
      
			for (j = 0; j <= njnt; j++) {
				distances[j] += dmax;
			}
      
			du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                         FORWARD, distances);
			u += du;
			if (u > umax - EPS6) {
				end_localpath++;
			}
		}
		p3d_set_and_update_this_robot_conf(robotPt, qsave);
		p3d_destroy_config(robotPt, qsave);
		MY_FREE(distances, double, njnt + 1);
	}
  
	*Kpath = 1.0;
	return (valid);
}

/*---------------------------------------------------------------------------*/

/****************************************************/
/****************************************************/
/*   FUNCTIONS FOR LOCAL MANAGMENT OF IKSOL         */
/****************************************************/
/****************************************************/

/* Note :
 This function supposes that ncntrts in cntrt_manager is constat !
 There are problems if cntrts are set by the user interface
 */
/**
 * @brief initialise static arrays: st_iksol, st_niksol and st_ikSolConfig.
 * @param cntrt_manager the constraint manager
 */
void p3d_init_iksol(p3d_cntrt_management *cntrt_manager) {
	int i, j;
	st_iksol_size = cntrt_manager->ncntrts;
  st_nbIkSols = p3d_get_nb_ikSol(cntrt_manager);
	st_iksol = MY_ALLOC(int*, st_iksol_size);
	st_niksol = MY_ALLOC(int, st_iksol_size);
  st_ikSolConfig = MY_ALLOC(double **, st_iksol_size);
  for (i = 0; i < st_iksol_size; i++) {
    st_ikSolConfig[i] = MY_ALLOC(double*, (cntrt_manager->cntrts[i])->nbSol);
    st_iksol[i] = MY_ALLOC(int, (cntrt_manager->cntrts[i])->nbSol);
    for (j = 0; j < (cntrt_manager->cntrts[i])->nbSol; j++) {
      st_ikSolConfig[i][j] = MY_ALLOC(double, (cntrt_manager->cntrts[i])->npasjnts);
      st_iksol[i][j] = -1;
    }
    st_niksol[i] = -1;
  }
}

void p3d_realloc_iksol(p3d_cntrt_management *cntrt_manager) {
  int **iksol = NULL;
  int *niksol = NULL;
  double ***ikSolConfig = NULL;
  int old_st_iksol_size = st_iksol_size;
  
  iksol = MY_ALLOC(int*, old_st_iksol_size);
  niksol = MY_ALLOC(
                    int, old_st_iksol_size);
  ikSolConfig = MY_ALLOC(
                         double **, old_st_iksol_size);
  for (int i = 0; i < old_st_iksol_size; i++) {
    niksol[i] = st_niksol[i];
    ikSolConfig[i] = MY_ALLOC(double*, (cntrt_manager->cntrts[i])->nbSol);
    iksol[i] = MY_ALLOC(
                        int, (cntrt_manager->cntrts[i])->nbSol);
    for (int j = 0; j
         < (cntrt_manager->cntrts[i])->nbSol; j++) {
      iksol[i][j] = st_iksol[i][j];
      ikSolConfig[i][j] = MY_ALLOC(double, (cntrt_manager->cntrts[i])->npasjnts);
      for (int k = 0; k
           < (cntrt_manager->cntrts[i])->npasjnts; k++) {
        ikSolConfig[i][j][k]
        = st_ikSolConfig[i][j][k];
      }
    }
  }
  p3d_destroy_iksol(cntrt_manager);
  p3d_init_iksol(cntrt_manager);
  
  for (int i = 0; i < old_st_iksol_size; i++) {
    st_niksol[i] = niksol[i];
    for (int j = 0; j
         < (cntrt_manager->cntrts[i])->nbSol; j++) {
      st_iksol[i][j] = iksol[i][j];
      for (int k = 0; k
           < (cntrt_manager->cntrts[i])->npasjnts; k++) {
        st_ikSolConfig[i][j][k]
        = ikSolConfig[i][j][k];
      }
      MY_FREE(ikSolConfig[i][j], double, (cntrt_manager->cntrts[i])->npasjnts);
    }
    MY_FREE(iksol[i], int, (cntrt_manager->cntrts[i])->nbSol);
    MY_FREE(ikSolConfig[i]
            , double *, (cntrt_manager->cntrts[i])->nbSol);
  }
  MY_FREE(iksol, int*, old_st_iksol_size);
  MY_FREE(niksol, int, old_st_iksol_size);
  MY_FREE(ikSolConfig, double **,
          old_st_iksol_size);
}

/**
 * @brief reset the static arrays: st_iksol, st_niksol and st_ikSolConfig.
 * @param cntrt_manager the constraint manager
 */
void p3d_reset_iksol(
                     p3d_cntrt_management *cntrt_manager) {
  int i = 0, j = 0, k = 0;
  
  for (i = 0; i < cntrt_manager->ncntrts; i++) {
    for (j = 0; j
         < (cntrt_manager->cntrts[i])->nbSol; j++) {
      for (k = 0; k
           < (cntrt_manager->cntrts[i])->npasjnts; k++) {
        st_ikSolConfig[i][j][k]
        = P3D_HUGE;
      }
      st_iksol[i][j] = -1;
    }
    st_niksol[i] = -1;
  }
}

/**
 * @brief copy the st_iksol vector.
 * @param cntrt_manager the constraint manager
 * @param iksol the returned vector
 */
void p3d_get_iksol_vector(
                          p3d_cntrt_management *cntrt_manager,
                          int ***iksol) {
  int i, j;
  
  if (st_iksol == NULL) {
    *iksol = NULL;
  } else {
    if (*iksol == NULL) {
      *iksol = MY_ALLOC(int*, cntrt_manager->ncntrts);
      for (i = 0; i
           < cntrt_manager->ncntrts; i++) {
        (*iksol)[i] = MY_ALLOC(int, (cntrt_manager->cntrts[i])->nbSol);
      }
    }
    for (i = 0; i < cntrt_manager->ncntrts; i++) {
      for (j = 0; j
           < (cntrt_manager->cntrts[i])->nbSol; j++) {
        (*iksol)[i][j]
        = st_iksol[i][j];
      }
    }
  }
}

/**
 * @brief copy the st_niksol vector.
 * @param cntrt_manager the constraint manager
 * @return the st_niksol vector
 */
int* p3d_get_niksol_vector(
                           p3d_cntrt_management *cntrt_manager) {
  return st_niksol;
}

/**
 * @brief copy the st_SolConfig vector.
 * @param cntrt_manager the constraint manager
 * @return return the st_SolConfig vector
 */
double*** p3d_get_ikSolConfig_vector(
                                     p3d_cntrt_management *cntrt_manager) {
  return st_ikSolConfig;
}

/**
 * @brief get the ith solution class for the given constraint number.
 * @param cntrt_manager the constraint manager
 * @param ctNum the constraint number
 * @param sol the position (i) of the solution class wanted
 * @return the solution class
 */
int p3d_get_ikSpecific_solution(
                                p3d_cntrt_management *cntrt_manager,
                                int ctNum, int sol) {
  return st_iksol[ctNum][sol];
}

/**
 * @brief get the passive jnt value for the given constraint and solution
 * @param cntrt_manager the constraint manager
 * @param ctNum the constraint number
 * @param sol the position (i) of the solution class wanted
 * @return Vector containing the passive DoF values
 */
double* p3d_get_ikSpecific_config(
                                  p3d_cntrt_management *cntrt_manager,
                                  int ctNum, int sol) {
  return st_ikSolConfig[ctNum][sol];
}

/**
 * @brief get the solution class number for in the ith position of the array st_iksol.
 * @param cntrt_manager the constraint manager
 * @param iksol_dst destination vector
 * @param sol the position (i) of the solution class wanted
 */
void p3d_get_iksol_vector_for_solution(
                                       p3d_cntrt_management *cntrt_manager,
                                       int **iksol_dst, int sol) {
  int i;
  if (!iksol_dst)
    printf("function : p3d_get_iksol_vector_for_solution, iksol_dst not intialised !!!\n");
  for (i = 0; i < cntrt_manager->ncntrts; i++)
    (*iksol_dst)[i]
    = st_iksol[i][sol];
}

/**
 * @brief check if there is multisolutions constraints
 * @param cntrt_manager the constraint manager
 * @param nbSol the maximum number of solutions
 * @return the number of multisol constraint
 */
int p3d_is_multisol(p3d_cntrt_management *cntrt_manager, int *nbSol) {
  int i = 0, j = 0;
  *nbSol = 1;
  for (i = 0; i < cntrt_manager->ncntrts; i++) {
    if ((cntrt_manager->cntrts[i])->nbSol
        > 1) {
      *nbSol *= (cntrt_manager->cntrts[i])->nbSol;
      j++;
    }
  }
  return j;
}


/**
 * @brief Check if the passed ikSol is a singularity.
 * @param cntrt_manager the constraint manager
 * @param ikSol the ikSol to check
 * @return true if the ikSol represent a singularity false otherwise.
 */
static int p3d_is_singularity(
                              p3d_cntrt_management *cntrt_manager,
                              int *ikSol) {
  for (int i = 0; i < cntrt_manager->ncntrts; i++) {
    if (ikSol[i] < 0) {
      return 1;
    }
  }
  return 0;
}

/**
 * @brief set the value of the first element of the st_iksol
 * @param ctNum the constraint number
 * @param val the value
 * @note DO not use
 */
void p3d_set_iksol_elem(int ctNum, int val) {
  if (st_iksol != NULL) {
    st_iksol[ctNum][0] = val;
  }
}

/**
 * @brief copy into the destination ik_sol the source ik_sol if isn't NULL else copy the st_iksol
 * @param cntrt_manager the constraint manager
 * @param iksol_src the iksol source
 * @param iksol_dst the destination ik_sol
 */
void p3d_copy_iksol(p3d_cntrt_management *cntrt_manager,
										int *iksol_src, int **iksol_dst) {
  int i = 0;
  
  if (iksol_src == NULL) {
    if (p3d_get_ik_choice() != IK_NORMAL) {
      *iksol_dst = MY_ALLOC(int, cntrt_manager->ncntrts);
      for (i = 0; i
           < cntrt_manager->ncntrts; i++)
        (*iksol_dst)[i]
        = st_iksol[i][0];
    }
  } else {
    *iksol_dst = MY_ALLOC(int, cntrt_manager->ncntrts);
    for (i = 0; i < cntrt_manager->ncntrts; i++)
      (*iksol_dst)[i]
      = iksol_src[i];
  }
}

/**
 * @brief compare the two given iksols. Two ik_sols are equal if all they member are equals or singular (-1)
 * @param cntrt_manager the constraint manager
 * @param iksol1 the first ik_sol
 * @param iksol2 the second ik_sol
 * @return True if the ik_sols are equals, False othewise.
 */
int p3d_compare_iksol(p3d_cntrt_management *cntrt_manager, int *iksol1, int *iksol2) {
  int i = 0;
  if (iksol1 != NULL && iksol2 != NULL) {
    if (p3d_is_singularity(cntrt_manager,iksol1) && p3d_is_singularity(cntrt_manager, iksol2)) {
      for (i = 0; i < cntrt_manager->ncntrts; i++) {
        if ((iksol1[i] > 0) && (iksol2[i] > 0)  && (iksol1[i] != iksol2[i]) ) {
              return 0;
            }
      }
    } else {
      for (i = 0; i < cntrt_manager->ncntrts; i++) {
        if ((iksol1[i] != iksol2[i]) && (iksol1[i] >= 0) && (iksol2[i] >= 0)) {
          return 0;
        }
      }
    }
  }
  return 1;
}

/**
 * @brief check if we can connect the two nodes (one of them or both are singular). For the singular node, we check if the nodes are in the same singularities classes.
 * @param rob the active robot
 * @param N1 the first node
 * @param N2 the second node
 * @return True if the two nodes can be connected, False otherwise
 */
int p3d_test_singularity_connexion(p3d_cntrt_management *cntrt_manager, p3d_node *N1, p3d_node *N2){
  int i = 0, j = 0, k = 0;
  p3d_singularity *singularity = NULL, *sing = NULL;
  p3d_singJntVal *singJntVal = NULL;
  p3d_jnt *jnt = NULL;
  p3d_cntrt * cntrt = NULL;
  double dof1 = 0.0, dof2 = 0.0;
  p3d_rob * r = XYZ_ROBOT;
  
  if (N1->isSingularity) {
    int singId = 0;
    singularity = p3d_get_config_singular_jnt(cntrt_manager, N1->q, &cntrt, &singId);//TODO simplifier en regardant simplement l'iksol et verifier
    if (singularity == NULL) {
      PrintError(("p3d_test_singularity_connexion: the node %d is taged singular but no singular value is found\n", N1->num));
      return FALSE;
    }
    for (i = 0; i < cntrt->nSingularities; i++) {
      sing = cntrt->singularities[i];
      for (j = 0; j < sing->nJnt; j++) {
        singJntVal = sing->singJntVal[j];
        jnt = r->joints[singJntVal->jntNum];
        for (k = 0; k < jnt->dof_equiv_nbr; k++) {
          dof1 = N1->q[jnt->index_dof +k];
          dof2 = N2->q[jnt->index_dof +k];
          if (((dof1 > singJntVal->val[k]) && (dof2 < singJntVal->val[k])) 
              || ((dof1 < singJntVal->val[k]) && (dof2 > singJntVal->val[k]))) {
              return FALSE;
          }
        }
      }
    }
  }
  
  if (N2->isSingularity) {
    int singId = 0;
    singularity = p3d_get_config_singular_jnt(cntrt_manager, N2->q, &cntrt, &singId);
    if (singularity == NULL) {
      PrintError(("p3d_test_singularity_connexion: the node %d is taged singular but no singular value is found\n", N2->num));
      return FALSE;
    }
    for (i = 0; i < cntrt->nSingularities; i++) {
      sing = cntrt->singularities[i];
      for (j = 0; j < sing->nJnt; j++) {
        singJntVal = sing->singJntVal[j];
        jnt = r->joints[singJntVal->jntNum];
        for (k = 0; k < jnt->dof_equiv_nbr; k++) {
          dof1 = N1->q[jnt->index_dof +k];
          dof2 = N2->q[jnt->index_dof +k];
          if (((dof1 > singJntVal->val[k]) && (dof2 < singJntVal->val[k]))
              || ((dof1 < singJntVal->val[k]) && (dof2 > singJntVal->val[k]))) {
              return FALSE;
          }
        }
      }
    }
  }
  return TRUE;
}

/**
 * @brief find in the given config the joint in singular position and the corresponding constraint.
 * @param cntrt_manager the constraint manager
 * @param q the config to check
 * @param cntrt the constraint controling the jnt in singular position
 * @param singId the singularity id in the cntrt array
 * @return the singularity
 */
p3d_singularity* p3d_get_config_singular_jnt(p3d_cntrt_management *cntrt_manager, configPt q, p3d_cntrt ** cntrt, int * singId) {
  int i = 0, j = 0, k = 0, l = 0,
  isSingularJnt = 1, nSingularValues =
  0, maxNJnt = 0;
  p3d_singularity *singularity = NULL,
  **singularValues = NULL;
  p3d_singJntVal *singJntVal = NULL;
  p3d_jnt* jnt = NULL;
  p3d_rob * r = XYZ_ROBOT;
  
  //for each constraint, check all specific jnt if one of them is in singular position.
  for (i = 0; i < cntrt_manager->ncntrts; i++) {
    (*cntrt) = cntrt_manager->cntrts[i];
    if ((*cntrt)->nSingularities) {
      singularValues = MY_ALLOC(p3d_singularity*, (*cntrt)->nSingularities);
      int *singulartyId = MY_ALLOC(int, (*cntrt)->nSingularities);
      for (j = 0; j < (*cntrt)->nSingularities; j++) {
        singularity = (*cntrt)->singularities[j];
        for (k = 0, isSingularJnt = 1; k < singularity->nJnt && isSingularJnt; k++) {
          singJntVal = singularity->singJntVal[k];
          jnt = r->joints[singJntVal->jntNum];
          for (l = 0; l < jnt->dof_equiv_nbr; l++) {//check all specific values (for each jnt DoF)
            if (q[jnt->index_dof + l] != singJntVal->val[l]) {
              isSingularJnt = 0;
              break;//it's not the jnt in singular position
            }
          }
          if (l == jnt->dof_equiv_nbr) {//if it's a singular jnt
            if (DEBUG_CNTRTS) {
              printf("jntSing = %d, values = ",jnt->num);
              for (l = 0; l< jnt->dof_equiv_nbr; l++) {
                printf("%f", q[jnt->index_dof+l]);
                if (l < jnt->dof_equiv_nbr - 1) {
                  printf(" ");
                }
              }
              printf("\n");
            }
          }
        }
        if (isSingularJnt) {//if all jnts of this singularity are in singular position
          singularValues[nSingularValues] = singularity;
          singulartyId[nSingularValues] = j;
          nSingularValues++;
        }
      }
      if (nSingularValues) {//if we have more than one singularity
        for (j = 0; j < nSingularValues; j++) {//select the singularity which have the greatest number of jnt
          if ((singularValues[j])->nJnt > maxNJnt) {
            singularity = singularValues[j];
            *singId = singulartyId[j];
          }
        }
        MY_FREE(singularValues, p3d_singularity*, (*cntrt)->nSingularities);
        return singularity;
      }
      MY_FREE(singularValues, p3d_singularity*, (*cntrt)->nSingularities);
    }
  }
  if (DEBUG_CNTRTS) {
    printf("Not a sing\n");
  }
  return NULL; //there is no jnt in singular position
}

/**
 * @brief extract from the the two ik_sols source a non singular iksol (i.e. no -1 in the destination ik_sol). This function is used tu générate for ex, a localpath ik_sol from its init and goal node ik_sols
 * @param cntrt_manager the constraint manager
 * @param iksol_src1 the first ik_sol
 * @param iksol_src2 the second ik_sol
 * @param iksol_dst the destination ik_sol
 */
void p3d_get_non_sing_iksol(
                            p3d_cntrt_management *cntrt_manager,
                            int *iksol_src1, int *iksol_src2,
                            int **iksol_dst) {
  int i = 0;
  
  if (iksol_src1 == NULL || iksol_src2
      == NULL) {
    //     p3d_copy_iksol(cntrt_manager, NULL, iksol_dst);
    //DO NOTHING
  } else {
    *iksol_dst = MY_ALLOC(int, cntrt_manager->ncntrts);
    for (i = 0; i < cntrt_manager->ncntrts; i++) {
      if (!p3d_compare_iksol(
                             cntrt_manager, iksol_src1,
                             iksol_src2)) {
        printf("Error The two iksols are differents\n");
        printf("iksol1\n");
        p3d_print_iksol(cntrt_manager, iksol_src1);
        printf("iksol2\n");
        p3d_print_iksol(cntrt_manager, iksol_src2);

        MY_FREE(*iksol_dst, int, cntrt_manager->ncntrts);
        p3d_copy_iksol(cntrt_manager,
                       NULL, iksol_dst);
        break;
      } else {
        if (iksol_src1[i] >= 0) {
          (*iksol_dst)[i]
          = iksol_src1[i];
        } else {
          (*iksol_dst)[i]
          = iksol_src2[i];
        }
      }
    }
  }
  return;
}

/**
 * @brief Compute the number of solution nbSolIk1*nbSolIk2*...*nbSolIkn
 * @param cntrt_manager the constraint mamanger
 * @return the number max of solutions.
 */
int p3d_get_nb_ikSol(p3d_cntrt_management *cntrt_manager) {
  int i = 0, nbSolutions = 0;
  
//  if (p3d_get_ik_choice() != IK_NORMAL) {
    for (i = 0, nbSolutions = 1; i
         < cntrt_manager->ncntrts; i++) {//compute the number of solutions.
      if (((cntrt_manager->cntrts[i])->nbSol
           > 1)
          && ((cntrt_manager->cntrts[i])->markedForSingularity
              != 1)) {
            nbSolutions
            *= (cntrt_manager->cntrts[i])->nbSol;
          }
    }
//  } else {
//    nbSolutions = 1;
//  }
  return nbSolutions;
}

/**
 * @brief set the current robot's iksol to the given ikSol value
 * @param r the current robot
 * @param ikSol the ikSol
 */

void p3d_set_robot_iksol(p3d_rob * r,
                         int *ikSol) {
  p3d_destroy_specific_iksol(
                             r->cntrt_manager, r->ikSol);
  p3d_copy_iksol(r->cntrt_manager, ikSol,
                 &(r->ikSol));
}

/**
 * @brief take a random solution class for the specified constraint.
 * @param cntrt_manager the constraint manager
 * @param ctNum the constraint number
 * @return the solution class
 */
int p3d_get_random_ikSol(
                         p3d_cntrt_management *cntrt_manager,
                         int ctNum) {
  return (int)p3d_random(1,
                         cntrt_manager->cntrts[ctNum]->nbSol
                         + 1);
}

/**
 * @brief take a random singularity from the constraint
 * @param ct the constraint
 * @return the singularity number in this constraint
 */
int p3d_get_random_singularity(p3d_cntrt *ct) {
  p3d_rob
  *r =
  (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int proba = 0, singularityNum = 0,
  *singList = NULL;
  
  if (p3d_get_planning_type() == P3D_SPECIFIC) {//specific planner
    proba = (int)p3d_random(0, 2);
    //     proba = 1;
  }
  if (proba == 1) {//use a passage singularity
    singList = MY_ALLOC(int, ct->nSingularities);
    for (int i = 0; i < ct->nSingularities; i++) {
      singList[i] = -1;
    }
    if (p3d_get_singularities_to_cross(ct,
                                       r->ikSolPos[ct->num],
                                       r->ikSolGoto[ct->num],
                                       &singList, 0)) {
      for (int i = 0; i
           < ct->nSingularities; i++) {
        if (singList[i] == -1) {
          i--;
          singularityNum
          = (int)p3d_random(
                            0, i);
          break;
        }
      }
      singularityNum
      = singList[singularityNum];
    }
    MY_FREE(singList, int, ct->nSingularities);
  } else {
    singularityNum = (int)p3d_random(0,
                                     ct->nSingularities);
  }
  
  return singularityNum;
}

/**
 * @brief Get the list of the singularities to cross to connect the startSol and the goalSol (only for one constraint.)
 * @param ct the conserned constraint
 * @param startSol the start ikSol
 * @param goalSol the goal ikSol
 * @param singList the list of singularities to cross
 * @param depth the depth for the recursion
 * @return true if we reach the goal ikSol false otherwise
 */
static int p3d_get_singularities_to_cross(
                                          p3d_cntrt *ct, int startSol,
                                          int goalSol, int **singList, int depth) {
  p3d_singularity * singularity = NULL;
  
  if (depth >= ct->nSingularities) {
    return FALSE;
  }
  for (int i = 0; i < ct->nSingularities; i++) {
    singularity = ct->singularities[i];
    for (int j = 0; j < singularity->nRel; j++) {
      if ((singularity->classes[j][0]
           == startSol
           && singularity->classes[j][1]
           == goalSol)
          || (singularity->classes[j][1]
              == startSol
              && singularity->classes[j][0]
              == goalSol)) {
            (*singList)[depth] = i;//put the singularity number
            return TRUE;
          }
    }
  }
  for (int i = 0; i < ct->nSingularities; i++) {
    singularity = ct->singularities[i];
    for (int j = 0; j < singularity->nRel; j++) {
      if (singularity->classes[j][0]
          == startSol) {
        
        if (p3d_get_singularities_to_cross(
                                           ct,
                                           singularity->classes[j][1],
                                           goalSol, singList,
                                           depth + 1)) {
          (*singList)[depth]
          = i;//put the singularity number
          return TRUE;
        }
      }
      if (singularity->classes[j][1]
          == startSol) {
        if (p3d_get_singularities_to_cross(
                                           ct,
                                           singularity->classes[j][0],
                                           goalSol, singList,
                                           depth + 1)) {
          (*singList)[depth]
          = i;//put the singularity number
          return TRUE;
        }
      }
    }
  }
  return FALSE;
}

/**
 * @brief print the given ik_sol
 * @param cntrt_manager the constraint mamanger
 * @param iksol the ik_sol to print
 */
void p3d_print_iksol(
                     p3d_cntrt_management *cntrt_manager,
                     int* iksol) {
  int i = 0;
  if (iksol) {
    PrintInfo(("Iksol = "));
    for (i = 0; i < cntrt_manager->ncntrts; i++) {
      PrintInfo(("%d", iksol[i]));
      if (i != cntrt_manager->ncntrts - 1) {//it isn't the last element
        PrintInfo((", "));
      }
    }
  } else {
    PrintInfo(("Iksol = "));
    for (i = 0; i < cntrt_manager->ncntrts; i++) {
      PrintInfo(("%d", st_iksol[i][0]));
      if (i != cntrt_manager->ncntrts - 1) {//it isn't the last element
        PrintInfo((", "));
      }
    }
  }
  PrintInfo(("\n"));
}

/**
 * @brief Check if the given config is close to sigularity. Close mean that |q[i] - i_sing| <= 5% * q[i]_range
 * @param cntrt_manager the constraint manager
 * @param the config to check
 * @param the singularity number if it close (> 0), 0 otherwise
 * @return the constraint num if it's close, -1 otherwise.
 */
int p3d_isCloseToSingularityConfig(p3d_rob* robot, p3d_cntrt_management *cntrt_manager, configPt config, int* singNum) {
  for(int i = 0; i < cntrt_manager->ncntrts; i++){
    p3d_cntrt* cntrt = cntrt_manager->cntrts[i];
    for(int j = 0; j < cntrt->nSingularities; j++){
      p3d_singularity* sing = cntrt->singularities[j];
      int near = TRUE;
      for(int k = 0; k < sing->nJnt; k++){
        p3d_jnt* jnt = robot->joints[sing->singJntVal[k]->jntNum];
        for(int h = 0; h < jnt->dof_equiv_nbr; h++){
          // compute dof range
          double range = jnt->dof_data[h].vmax - jnt->dof_data[h].vmin;
          if (ABS(config[jnt->index_dof + h] - sing->singJntVal[k]->val[h]) > (1 * range) / 100){//(0.0873 * range) / M_PI) {//5° for a 360° joint range
            near = FALSE;
            break;
          }else {
            //printf("near config = %f, sing = %f, range = %f\n", config[jnt->index_dof + h], sing->singJntVal[k]->val[h], 2 * range * 180 / (100*M_PI));
          }

        }
        if(near){
          *singNum = j;
          return i;
        }
      }
    }
  }
  return -1;
}

/**
 * @brief desalloc the static vectors st_niksol, st_iksol and st_ikSolConfig.
 * @param cntrt_manager the constraint mamanger
 */
void p3d_destroy_iksol(
                       p3d_cntrt_management *cntrt_manager) {
  int i, j;
  MY_FREE(st_niksol, int, st_iksol_size);
  st_niksol = NULL;
  if (cntrt_manager->cntrts) {
    for (i = 0; i < st_iksol_size; i++) {
      for (j = 0; j
           < (cntrt_manager->cntrts[i])->nbSol; j++) {
        MY_FREE(st_ikSolConfig[i][j], double , (cntrt_manager->cntrts[i])->npasjnts);
      }
      MY_FREE(st_iksol[i], int , (cntrt_manager->cntrts[i])->nbSol);
      MY_FREE(st_ikSolConfig[i]
              , double* , (cntrt_manager->cntrts[i])->nbSol);
    }
    MY_FREE(st_iksol, int, st_iksol_size);
    st_iksol = NULL;
    MY_FREE(st_ikSolConfig, double **,
            st_iksol_size);
    st_ikSolConfig = NULL;
  }
}

/**
 * @brief desalloc the given vector.
 * @param cntrt_manager the constraint manager
 * @param iksol the vector to destroy
 */
void p3d_destroy_specific_iksol(
                                p3d_cntrt_management *cntrt_manager,
                                int *iksol) {
  MY_FREE(iksol, int, cntrt_manager->ncntrts);
}

/**
 * @brief desalloc the given vector.
 * @param cntrt_manager the constraint manager
 * @param niksol the vector to destroy
 */
void p3d_destroy_specific_niksol(
                                 p3d_cntrt_management *cntrt_manager,
                                 int *niksol) {
  MY_FREE(niksol, int, cntrt_manager->ncntrts);
  niksol = NULL;
}

/**
 * @brief mark the given constraint for singularity.
 * @param cntrt_manager the constraint manager
 * @param ctNum the constraint number
 */
void p3d_mark_for_singularity(
                              p3d_cntrt_management *cntrt_manager,
                              int ctNum) {
  cntrt_manager->cntrts[ctNum]->active = 0; //disable the constraint.
  cntrt_manager->cntrts[ctNum]->markedForSingularity
  = 1; //set the constraint ready for singularity.
}

/**
 * @brief unmark the given constraint for singularity
 * @param cntrt_manager the constraint manager
 * @param ctNum the constraint number
 */
void p3d_unmark_for_singularity(
                                p3d_cntrt_management *cntrt_manager,
                                int ctNum) {
  cntrt_manager->cntrts[ctNum]->active = 1; //enable the constraint.
  cntrt_manager->cntrts[ctNum]->markedForSingularity
  = 0;
}

/**
 * @brief Check if the given IkSol is inside the given array
 * @param cntrt_manager the constraint manager
 * @param ikSol The ikSol to find
 * @param ikSolArray The ikSol Array to search into
 * @return true if the ikSol is found, false otherwise
 */
int p3d_findIkSolInArray (p3d_cntrt_management* cntrt_manager, int* ikSol, int ** ikSolArray, int nbArrayItems){
  for(int i =  0; i < nbArrayItems; i++){
    if(p3d_compare_iksol(cntrt_manager, ikSol, ikSolArray[i])){
      return TRUE;
    }
  }
  return FALSE;
}

/**
 * @brief Check if the given IkSol is inside the given array and add it if do not exist
 * @param cntrt_manager the constraint manager
 * @param ikSol The ikSol to find
 * @param ikSolArray The ikSol Array to search into
 * @param nbArrayItems The number of items in ikSolArray
 * @return true if it added, false otherwise
 */
int p3d_AddIkSolInArray (p3d_cntrt_management* cntrt_manager, int* ikSol, int ** ikSolArray, int *nbArrayItems){
  if(ikSol && !p3d_findIkSolInArray(cntrt_manager, ikSol, ikSolArray, *nbArrayItems)){
    for(int i = 0; i < st_iksol_size; i++){
      ikSolArray[*nbArrayItems][i] = ikSol[i];
    }
    (*nbArrayItems)++;
    printf("added ");
    p3d_print_iksol(cntrt_manager, ikSol);
    return TRUE;
  }
  return FALSE;
}

/** \brief this function find a constraint where the given joint number is passive
 \param r The current robot
 \param joint the joint number
 \return the constraint if found Null otherwise
 */
p3d_cntrt** p3d_getJointCntrts(
                               p3d_cntrt_management * cntrt_manager,
                               int joint, int *nbCntrts) {
  p3d_cntrt** cntrts = NULL;
  p3d_cntrt* ct = NULL;
  *nbCntrts = 0;
  for (int i = 0; i < cntrt_manager->ncntrts; i++) {
    ct = cntrt_manager->cntrts[i];
    for (int j = 0; j < ct->npasjnts; j++) {
      if ((ct->pasjnts[j])->num
          == joint) {
        if (cntrts == NULL) {
          cntrts = MY_ALLOC(p3d_cntrt*, 1);
        } else {
          cntrts = MY_REALLOC(cntrts, p3d_cntrt*, *nbCntrts , *nbCntrts + 1);
        }
        cntrts[*nbCntrts] = ct;
        (*nbCntrts)++;
      }
    }
  }
  return cntrts;
}

/**********************************************************/
int p3d_local_conf_correction(p3d_rob *robotPt,
                              configPt q) {
  double val;
  int njnt = robotPt->njoints, i, j, k;
  double vmin, vmax, vari;
  double dvmin, dvmax;
  p3d_jnt * jntPt;
  p3d_jnt *Jlaj, *Jfaj;
  int ntry;
  configPt qp;
  double dl = 0.0; // WARNING : unused by current cntrt functions
  p3d_cntrt *ct;
  pp3d_rlg_chain rlgchPt;
  int * ikSol = NULL;
  if (p3d_get_ik_choice() != IK_NORMAL) {
    ikSol = MY_ALLOC(int, robotPt->cntrt_manager->ncntrts);
  }
  qp = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt, q, &qp);
  
  // try to generate valid conformation a number of times
  for (ntry = 0; ntry < 1000; ntry++) {
    // shoot
    
    // if loop(s)
    if (p3d_get_RLG()) {
      if (robotPt->cntrt_manager->cntrts
          != NULL) {
        for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list); dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
          ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
          if ((ct->active)
              && (ct->rlgPt
                  != NULL)
              && (ct->rlgPt->rlgchPt
                  != NULL)) {
                rlgchPt
                = ct->rlgPt->rlgchPt;
                Jfaj
                = rlgchPt->rlg_data[0]->jnt;
                Jlaj
                = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;
                jntPt = Jfaj;
                while ((jntPt != NULL)
                       && (jntPt->prev_jnt
                           != Jlaj)) {
                         for (j = 0; j
                              < jntPt->dof_equiv_nbr; j++) {
                           k
                           = jntPt->index_dof
                           + j;
                           p3d_jnt_get_dof_rand_bounds(
                                                       jntPt,
                                                       j,
                                                       &vmin,
                                                       &vmax);
                           val
                           = p3d_jnt_get_dof(
                                             jntPt,
                                             j);
                           // allowed variation = +-10%
                           //vari = (vmax - vmin) / 20;
                           vari
                           = (vmax
                              - vmin)
                           * (double)ntry
                           / (1000.0
                              * 10.0);
                           dvmax = val
                           + vari;
                           dvmin = val
                           - vari;
                           if (dvmax
                               > vmax)
                             dvmax
                             = vmax;
                           if (dvmin
                               < vmin)
                             dvmin
                             = vmin;
                           q[k]
                           = p3d_random(
																				dvmin,
																				dvmax);
                           jntPt
                           = jntPt->next_jnt[jntPt->n_next_jnt - 1];
                           if ((jntPt
                                == NULL)
                               || (jntPt->prev_jnt
                                   == Jlaj))
                             break;
                         }
                       }
              }
        }
      }
    }
    
    // if other constraints
    else {
      for (i = 0; i <= njnt; i++) {
        jntPt = robotPt->joints[i];
        for (j = 0; j
             < jntPt->dof_equiv_nbr; j++) {
          k = jntPt->index_dof + j;
          if (p3d_jnt_get_dof_is_user(
                                      jntPt, j)
              // NEW : DO NOT PERTURB FREEFLYING JNTS
              && jntPt->type
              != P3D_FREEFLYER) {
            p3d_jnt_get_dof_rand_bounds(
                                        jntPt, j,
                                        &vmin, &vmax);
            val = p3d_jnt_get_dof(
                                  jntPt, j);
            // allowed variation = +-10%
            //vari = (vmax - vmin) / 20;
            vari = (vmax - vmin)
            * (double)ntry
            / (1000.0
               * 10.0);
            dvmax = val + vari;
            dvmin = val - vari;
            if (dvmax > vmax)
              dvmax = vmax;
            if (dvmin < vmin)
              dvmin = vmin;
            q[k] = p3d_random(
                              dvmin, dvmax);
          } else {
            q[k] = p3d_jnt_get_dof(
                                   jntPt, j);
          }
        }
      }
    }
    if (p3d_get_ik_choice() != IK_NORMAL) {
      for (j = 0; j
           < robotPt->cntrt_manager->ncntrts; j++) {
        p3d_set_iksol_elem(
                           j,
                           p3d_get_random_ikSol(
                                                robotPt->cntrt_manager,
                                                j));
      }
    }
    if (p3d_set_and_update_this_robot_conf_multisol(
                                                    robotPt, q, qp, dl, ikSol)) {
      p3d_destroy_config(robotPt, qp);
      return (TRUE);
    }
  }
  p3d_destroy_config(robotPt, qp);
  if (p3d_get_ik_choice() != IK_NORMAL) {
    MY_FREE(ikSol, int, robotPt->cntrt_manager->ncntrts);
  }
  return (FALSE);
}
/**********************************************************/

p3d_cntrt * getJntFixedCntrt(
                             p3d_cntrt_management * cntrt_manager,
                             int jntNum) {
  int nbCntrts = 0;
  p3d_cntrt** cntrts = p3d_getJointCntrts(
                                          cntrt_manager, jntNum, &nbCntrts);
  for (int i = 0; i < nbCntrts; i++) {
    if (strcmp(cntrts[i]->namecntrt,
               "p3d_fixed_jnt") == 0
        && cntrts[i]->pasjnts[0]->num
        == jntNum) {
      return cntrts[i];
    }
  }
  return NULL;
}

#if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
//! Creates constraints corresponding to the inverse of the closed chained constraints of the robot.
//! This is used to automatically update the pose of virtual objects.
//! \return 0 in case of success, 1 otherwise
p3d_cntrt* p3d_create_FK_cntrts(p3d_rob* robotPt, p3d_cntrt* armCntrt){
  p3d_cntrt* fkCntrt = setAndActivateTwoJointsFixCntrt(robotPt, armCntrt->actjnts[0], armCntrt->pasjnts[armCntrt->npasjnts-1]);
  p3d_matInvertXform(armCntrt->Tatt, fkCntrt->Tatt);
  p3d_desactivateCntrt(robotPt, fkCntrt);
  return fkCntrt;
} 
#endif
