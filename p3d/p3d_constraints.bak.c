/* (jcortes) */
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"

// FOR DEBUGGING //
#define DEBUG_CNTRTS 0
///////////////////

#define sqr(x) ((x)*(x))

#define DOF_WITHOUT_CNTRT 0
#define DOF_PASSIF 2
#define DOF_ACTIF  1

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

// NOTE :
//        the treatment if several IK solutions is not working yet
//        but functions are prepared for it

static int *st_iksol = NULL;
static int *st_niksol = NULL;
static int look_iksol = 0;
static void alloc_and_init_st_iksol(p3d_cntrt_management *cntrt_manager);

static int p3d_set_fixed_dof(p3d_cntrt_management * cntrt_manager,
			     p3d_jnt **pas_jntPt, int *pas_jnt_dof, 
			     int *pas_rob_dof, p3d_jnt **act_jntPt,
			     int *act_jnt_dof, int *act_rob_dof,
			     double val, int ct_num, int state);
static int p3d_set_lin_rel_dofs(p3d_cntrt_management * cntrt_manager,
				p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				double k,  double C, int ct_num, int state);
static int p3d_set_rel_dofs(p3d_cntrt_management * cntrt_manager,
			    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			    double k1, double k2, double k3, double k4, int ct_num, int state);
static int p3d_set_jnt_on_ground(p3d_cntrt_management * cntrt_manager, 
				 int nb_pas,
				 p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				 p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				 int nb_Dval, double* Dval, int *Ival, int ct_num, int state);
static int p3d_set_4Rlnk(p3d_cntrt_management * cntrt_manager,
			 p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			 p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			 int ct_num, int state);
static int p3d_set_RRPRlnk(p3d_cntrt_management * cntrt_manager,
			    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			    int ct_num, int state);
static int p3d_set_P3Rlnk(p3d_cntrt_management * cntrt_manager,                      /* << modif EF pour Delmia */
			   p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			   p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			   int ct_num, int state);                                  
static int p3d_set_3RPRlnk(p3d_cntrt_management * cntrt_manager,
			   p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			   p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			   int ct_num, int state);



static int p3d_set_car_front_wheels(p3d_cntrt_management * cntrt_manager,
				    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				    double h, double a, int ct_num, int state);
static int p3d_set_cycab_wheels(p3d_cntrt_management * cntrt_manager,
				p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				double l1, double l2, double e, int ct_num, int state);
static int p3d_set_planar_closed_chain(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       int whatcase, int ct_num, int state);



static int p3d_fct_fixed_jnt(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_lin_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_jnt_on_ground(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_4Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_RRPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_P3Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl); /* << modif EF pour Delmia */
static int p3d_fct_3RPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_car_front_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_cycab_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl);
static int p3d_fct_planar_closed_chain(p3d_cntrt *ct, int iksol, configPt qp, double dl);


/* -- functions for RLG test -- */
static int p3d_set_in_sphere(p3d_cntrt_management * cntrt_manager,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double x, double y, double z, int ct_num, int state);
static int p3d_fct_in_sphere(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */
static int p3d_set_jnts_relpos_bound(p3d_cntrt_management * cntrt_manager,
				     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				     double *Dval, int *Ival, int ct_num, int state);
static int p3d_fct_jnts_relpos_bound(p3d_cntrt *ct, int iksol, configPt qp, double dl);


/* -- functions for 3R IK -- */
static int p3d_set_3R_arm_ik(p3d_cntrt_management * cntrt_manager,int nb_pas,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double *Dval, int *Ival, int ct_num, int state);
static int p3d_fct_3R_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for R6 IK -- */
static int p3d_set_R6_arm_ik(p3d_cntrt_management * cntrt_manager,int nb_pas,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double *Dval, int *Ival, int ct_num, int state);
static int p3d_fct_R6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);


/* -- functions for prismatic actuator IK -- */
static int p3d_set_prismatic_actuator_ik(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       double *Dval, int ct_num, int state);
static int p3d_fct_prismatic_actuator_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_prismatic_actuator_II_ik(p3d_cntrt_management * cntrt_manager,
					    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
					    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
					    double l0, int ct_num, int state);
static int p3d_fct_prismatic_actuator_II_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

/* -- functions for 6R_bio_ik -- */
static int p3d_set_6R_bio_ik(p3d_cntrt_management * cntrt_manager,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     int ct_num, int state);
static int p3d_fct_6R_bio_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_6R_bio_ik_nopep(p3d_cntrt_management * cntrt_manager,
				   p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				   p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				   int ct_num, int state);
static int p3d_fct_6R_bio_ik_nopep(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_6R_bio_ik_nopep_new(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       double *Dval, int ct_num, int state);
static int p3d_fct_6R_bio_ik_nopep_new(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_bio_bkb_Hbond_cntrt(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       double *Dval, int *Ival, int ct_num, int state);
static int p3d_fct_bio_bkb_Hbond_cntrt(p3d_cntrt *ct, int iksol, configPt qp, double dl);

static int p3d_set_bio_diS_bond_cntrt(p3d_cntrt_management * cntrt_manager,
				      p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				      p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				      double *Dval, int *Ival, int ct_num, int state);
static int p3d_fct_bio_diS_bond_cntrt(p3d_cntrt *ct, int iksol, configPt qp, double dl);



/* -- functions for CTBot IK -- */
static int p3d_set_CTBot_ik(p3d_cntrt_management * cntrt_manager, 
			    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			    double *Dval, int ct_num, int state);
static int p3d_fct_CTBot_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl);



static p3d_cntrt *last_cntrt_set = NULL;


/*---------------------------------------------------------------------------*/
static int TEST_PHASE = 0;

void p3d_set_TEST_PHASE(int val)
{
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
				int *nb_Dofactiv, int *nb_Dval, int *nb_Ival)
{
  *nb_Dofpasiv = 0;
  *nb_Dofactiv = 0;
  *nb_Dval = 0;
  *nb_Ival = 0;
  
  if(strcmp(namecntrt, CNTRT_FIXED_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dval = 1;
  } else if(strcmp(namecntrt, CNTRT_LIN_REL_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 1;
    *nb_Dval = 2;
  } else if(strcmp(namecntrt, CNTRT_REL_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 2;
    *nb_Dval = 4;
  } else if(strcmp(namecntrt, CNTRT_ON_GROUND_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dval = 2;
    *nb_Ival = 3;
  } else if(strcmp(namecntrt, CNTRT_RRPR_NAME)==0) {
    *nb_Dofpasiv = 2;
    *nb_Dofactiv = 1;
  } else if(strcmp(namecntrt, CNTRT_4R_NAME)==0) {
    *nb_Dofpasiv = 3;
    *nb_Dofactiv = 1;
  } else if(strcmp(namecntrt, CNTRT_P3R_NAME)==0) {
    *nb_Dofpasiv = 2;
    *nb_Dofactiv = 1;
  } else if(strcmp(namecntrt, CNTRT_3RPR_NAME)==0) {
    *nb_Dofpasiv = 2;
    *nb_Dofactiv = 2;
  } else if(strcmp(namecntrt, CNTRT_CAR_FRONT_WHEELS_NAME)==0) {
    *nb_Dofpasiv = 2;
    *nb_Dofactiv = 1;
    *nb_Dval     = 2;
  } else if(strcmp(namecntrt, CNTRT_CYCAB_WHEELS_NAME)==0) {
    *nb_Dofpasiv = 4;
    *nb_Dofactiv = 1;
    *nb_Dval     = 3;
  } else if(strcmp(namecntrt, CNTRT_PLANAR_CLOSED_CHAIN_NAME)==0) {
    *nb_Dofpasiv = 4;
    *nb_Dofactiv = 1;
    *nb_Ival     = 1;
  } else if(strcmp(namecntrt, CNTRT_IN_SPHERE_NAME)==0) {
    *nb_Dval     = 3;
  } else if(strcmp(namecntrt, CNTRT_JNTS_RELPOS_BOUND)==0) {
    *nb_Dofpasiv = 0;
    *nb_Dofactiv = 0;
    *nb_Dval     = 2;
    *nb_Ival     = 2;
  } else if(strcmp(namecntrt,CNTRT_3R_ARM_NAME)==0) {
    *nb_Dofpasiv = 3;
    *nb_Dofactiv = 1;
    *nb_Dval     = 3;
    *nb_Ival     = 1;
  } else if(strcmp(namecntrt,CNTRT_R6_ARM_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 1;
    *nb_Dval     = 3;
    *nb_Ival     = 3;
  } else if(strcmp(namecntrt,CNTRT_PRISMATIC_ACTUATOR_NAME)==0) {
    *nb_Dofpasiv = 3;
    *nb_Dofactiv = 1;
    *nb_Dval     = 3;
    *nb_Ival     = 0;
  } else if(strcmp(namecntrt,CNTRT_PRISMATIC_ACTUATOR_II_NAME)==0) {
    *nb_Dofpasiv = 3;
    *nb_Dofactiv = 1;
    *nb_Dval     = 1;
    *nb_Ival     = 0;
  } else if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 1;
    *nb_Dval     = 0;
    *nb_Ival     = 0;
  } else if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME_NOPEP)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 1;
    *nb_Dval     = 0;
    *nb_Ival     = 0;
  } else if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME_NOPEP_NEW)==0) {
    *nb_Dofpasiv = 1;
    *nb_Dofactiv = 1;
    *nb_Dval     = 12;
    *nb_Ival     = 0;
  } else if(strcmp(namecntrt,CNTRT_BIO_BKB_HBOND)==0) {
    *nb_Dofpasiv = 0;
    *nb_Dofactiv = 0;
    *nb_Dval     = 2;
    *nb_Ival     = 4;
  } else if(strcmp(namecntrt,CNTRT_BIO_DIS_BOND)==0) {
    *nb_Dofpasiv = 0;
    *nb_Dofactiv = 0;
    *nb_Dval     = 2;
    *nb_Ival     = 2;
  } else if(strcmp(namecntrt,CNTRT_CTBOT_NAME)==0) {
    *nb_Dofpasiv = 13;
    *nb_Dofactiv = 1;
    *nb_Dval     = 0;
    *nb_Ival     = 0;
  } 
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
int p3d_create_constraint(
	       p3d_cntrt_management * cntrt_manager, const char *namecntrt, 
	       int nb_pas, p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *Dofpassiv,
	       int nb_act, p3d_jnt **act_jntPt,int *act_jnt_dof,int *Dofactiv,
	       int nb_Dval, double *Dval, int nb_Ival, int *Ival,
	       int ct_num, int state)
{
  if ((ct_num>=0) && (ct_num>cntrt_manager->ncntrts)) {
    PrintWarning(("ERROR: s_p3d_create_constraint: wrong constraint num\n"));
    return(FALSE);    
  }
  if(strcmp(namecntrt, CNTRT_FIXED_NAME)==0) {
    return p3d_set_fixed_dof(cntrt_manager, pas_jntPt, pas_jnt_dof, Dofpassiv, 
			     act_jntPt, act_jnt_dof, Dofactiv, 
			     Dval[0], ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_LIN_REL_NAME)==0) {
    return p3d_set_lin_rel_dofs(cntrt_manager, pas_jntPt, pas_jnt_dof,
			       Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			       Dval[0],Dval[1], ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_ON_GROUND_NAME)==0) {
    return p3d_set_jnt_on_ground(cntrt_manager, nb_pas, pas_jntPt, pas_jnt_dof,
				 Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				 nb_Dval, Dval, Ival, ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_RRPR_NAME)==0) {
    return p3d_set_RRPRlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
			   Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			   ct_num,state);
  }
  if(strcmp(namecntrt, CNTRT_4R_NAME)==0) {
    return p3d_set_4Rlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
			 Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			 ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_P3R_NAME)==0) {
    return p3d_set_P3Rlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
			  Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			  ct_num,state);
  }
  if(strcmp(namecntrt, CNTRT_3RPR_NAME)==0) {
    return p3d_set_3RPRlnk(cntrt_manager, pas_jntPt, pas_jnt_dof,
			   Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			   ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_CAR_FRONT_WHEELS_NAME)==0) {
    return p3d_set_car_front_wheels(cntrt_manager, pas_jntPt, pas_jnt_dof,
			    Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			    Dval[0], Dval[1], ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_CYCAB_WHEELS_NAME)==0) {
    return p3d_set_cycab_wheels(cntrt_manager, pas_jntPt, pas_jnt_dof,
				Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				Dval[0], Dval[1], Dval[2], ct_num, state);
  }
  if(strcmp(namecntrt, CNTRT_PLANAR_CLOSED_CHAIN_NAME)==0) {
    return p3d_set_planar_closed_chain(cntrt_manager, pas_jntPt, pas_jnt_dof,
				       Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				       Ival[0], ct_num,state);
  }
  /* -- functions for RLG test -- */
  if(strcmp(namecntrt, CNTRT_IN_SPHERE_NAME)==0) {
    return p3d_set_in_sphere(cntrt_manager, pas_jntPt, pas_jnt_dof,
   			     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
   			     Dval[0], Dval[1], Dval[2], ct_num,state);
  }  
  /* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */
  if(strcmp(namecntrt, CNTRT_JNTS_RELPOS_BOUND)==0) {
    return p3d_set_jnts_relpos_bound(cntrt_manager, pas_jntPt, pas_jnt_dof,
				     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				     Dval, Ival, ct_num,state);
  }  
  /* -- functions for 3R IK -- */
  if(strcmp(namecntrt,CNTRT_3R_ARM_NAME)==0) {
    return p3d_set_3R_arm_ik(cntrt_manager, nb_pas, pas_jntPt, pas_jnt_dof,
   			     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
   			     Dval, Ival, ct_num, state);
  }  
  /* -- functions for R6 IK -- */
  if(strcmp(namecntrt,CNTRT_R6_ARM_NAME)==0) {
    return p3d_set_R6_arm_ik(cntrt_manager, nb_pas, pas_jntPt, pas_jnt_dof,
   			     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
   			     Dval, Ival, ct_num, state);
  }  
  /* -- functions for prismatic actuator IK -- */
  if(strcmp(namecntrt,CNTRT_PRISMATIC_ACTUATOR_NAME)==0) {
    return p3d_set_prismatic_actuator_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
				       Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				       Dval, ct_num, state);
  }  
  if(strcmp(namecntrt,CNTRT_PRISMATIC_ACTUATOR_II_NAME)==0) {
    return p3d_set_prismatic_actuator_II_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
					    Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
					    Dval[0],ct_num, state);
  }  
  /* -- functions for 6R_bio_ik -- */
  if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME)==0) {
    return p3d_set_6R_bio_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
			     Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			     ct_num, state);  
  }  
  if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME_NOPEP)==0) {
    return p3d_set_6R_bio_ik_nopep(cntrt_manager, pas_jntPt, pas_jnt_dof,
				   Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				   ct_num, state);  
  }  
  if(strcmp(namecntrt,CNTRT_6R_BIO_IK_NAME_NOPEP_NEW)==0) {
    return p3d_set_6R_bio_ik_nopep_new(cntrt_manager, pas_jntPt, pas_jnt_dof,
				       Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				       Dval, ct_num, state);  
  }
  if(strcmp(namecntrt,CNTRT_BIO_BKB_HBOND)==0) {
    return p3d_set_bio_bkb_Hbond_cntrt(cntrt_manager, pas_jntPt, pas_jnt_dof,
				       Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				       Dval, Ival, ct_num, state);  
  }  
  if(strcmp(namecntrt,CNTRT_BIO_DIS_BOND)==0) {
    return p3d_set_bio_diS_bond_cntrt(cntrt_manager, pas_jntPt, pas_jnt_dof,
				      Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
				      Dval, Ival, ct_num, state);  
  }  
 /* -- functions for CTBot IK -- */
  if(strcmp(namecntrt,CNTRT_CTBOT_NAME)==0) {
    return p3d_set_CTBot_ik(cntrt_manager, pas_jntPt, pas_jnt_dof,
			    Dofpassiv, act_jntPt, act_jnt_dof, Dofactiv,
			    Dval, ct_num, state);
  }

  /* ---------------------- */
  PrintWarning(("ERROR: p3d_create_constraint: wrong constraint name\n"));
  return(FALSE);
}



/*--------------------------------------------------------------------------*/
/*! \brief Function to update a constraint.
 *
 * \param ct:    The constraint.
 * \param state: The state (1 active).
 *
 * \return TRUE if success, FALSE if it fails.
 */
int p3d_update_constraint(p3d_cntrt * ct, int state)
{
  return p3d_create_constraint(ct->cntrt_manager, ct->namecntrt, ct->npasjnts,
			       ct->pasjnts, ct->pas_jnt_dof, ct->pas_rob_dof,
			       ct->nactjnts, ct->actjnts, ct->act_jnt_dof,
			       ct->act_rob_dof, ct->ndval, ct->argu_d,
			       ct->nival, ct->argu_i, ct->num, state);
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
int p3d_constraint_dof(const char *namecntrt, 
	       int nb_pas, int *pas_jnt_num, int *pas_jnt_dof, 
	       int nb_act, int *act_jnt_num, int *act_jnt_dof,
	       int nb_Dval, double *Dval, int nb_Ival, int *Ival,
	       int ct_num, int state)
{
  p3d_rob *rPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  return p3d_constraint_dof_r(rPt, namecntrt, nb_pas, pas_jnt_num, pas_jnt_dof,
			      nb_act, act_jnt_num,act_jnt_dof, nb_Dval, Dval,
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
int p3d_constraint_dof_r(p3d_rob *robotPt, const char *namecntrt,
	       int nb_pas, int *pas_jnt_num, int *pas_jnt_dof, 
	       int nb_act, int *act_jnt_num, int *act_jnt_dof,
	       int nb_dval, double *Dval, int nb_ival, int *Ival,
	       int ct_num, int state)
{
  p3d_jnt * pas_jntPt[MAX_ARGU_CNTRT];
  p3d_jnt * act_jntPt[MAX_ARGU_CNTRT];  
  int Dofpassiv[MAX_ARGU_CNTRT];
  int Dofactiv[MAX_ARGU_CNTRT];
  int nb_passif, nb_actif, nb_Dval, nb_Ival, i;
  int valid;

  p3d_constraint_get_nb_param(namecntrt, &nb_passif, &nb_actif,
			      &nb_Dval, &nb_Ival);
  /* Check the number of parameters */
  if ((nb_passif != nb_pas) && (nb_pas != -1)) {
    valid = FALSE;
    if ((strcmp(namecntrt, CNTRT_R6_ARM_NAME) == 0) && (nb_pas == 6))
      { valid = TRUE; }
    if ((strcmp(namecntrt, CNTRT_ON_GROUND_NAME) == 0) && (nb_pas >= 0))
      { valid = TRUE; }
    if (!valid) {
      PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
		    "passif degree of freedom !!!\n"));
      return FALSE;
    }
    nb_passif = nb_pas;
  }
  if ((nb_actif != nb_act) && (nb_act != -1)) {
    PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
		  "actif degree of freedom !!!\n"));
    return FALSE;
  }
  if ((nb_Dval != nb_dval) && (nb_dval != -1)) {
    PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
		  "float parameters !!!\n"));
    return FALSE;
  }
  if ((nb_Ival != nb_ival) && (nb_ival != -1)) {
    PrintWarning(("ERROR: p3d_constraint_dof_r: wrong number of "
		  "integer parameters !!!\n"));
    return FALSE;
  }
  
  for(i=0; i<nb_passif; i++) {
    if ((pas_jnt_num[i]<0) || (pas_jnt_num[i]>robotPt->njoints)) {
      PrintWarning(("ERROR: p3d_constraint_dof: joint not valid !!!\n"));
      return FALSE;
    }
    pas_jntPt[i] = robotPt->joints[pas_jnt_num[i]];
    if ((pas_jnt_dof[i]<0) || (pas_jnt_dof[i]>pas_jntPt[i]->dof_equiv_nbr)) {
      PrintWarning(("ERROR: p3d_constraint_dof: dof not valid !!!\n"));
      return FALSE;
    }
    Dofpassiv[i] = pas_jntPt[i]->index_dof + pas_jnt_dof[i];
  }
  for(i=0; i<nb_actif; i++) {
    if ((act_jnt_num[i]<0) || (act_jnt_num[i]>robotPt->njoints)) {
      PrintWarning(("ERROR: p3d_constraint_dof: joint not valid !!!\n"));
      return FALSE;
    }
    act_jntPt[i] = robotPt->joints[act_jnt_num[i]];
    if ((act_jnt_dof[i]<0) || (act_jnt_dof[i]>act_jntPt[i]->dof_equiv_nbr)) {
      PrintWarning(("ERROR: p3d_constraint_dof: dof not valid !!!\n"));
      return FALSE;
    }
    Dofactiv[i] = act_jntPt[i]->index_dof + act_jnt_dof[i];
  }
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
int p3d_constraint(const char *namecntrt,
	       int nb_pas, int *Jpasiv,
	       int nb_act, int *Jactiv,
	       int nb_dval, double *Dval, int nb_ival, int *Ival,
	       int ct_num, int state)
{
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
int p3d_constraint_r(p3d_rob *robotPt, const char *namecntrt,
	       int nb_pas, int *Jpasiv,
	       int nb_act, int *Jactiv,
	       int nb_dval, double *Dval, int nb_ival, int *Ival,
	       int ct_num, int state)
{
  int jnt_dof_passif[MAX_ARGU_CNTRT];
  int jnt_dof_actif[MAX_ARGU_CNTRT];
  int i;

  for(i=0; i<MAX_ARGU_CNTRT; i++) {
    jnt_dof_passif[i] = 0;
    jnt_dof_actif[i] = 0;
  }

  return p3d_constraint_dof_r(robotPt, namecntrt, nb_pas, Jpasiv, 
			      jnt_dof_passif, nb_act, Jactiv, jnt_dof_actif,
			      nb_dval, Dval, nb_ival, Ival, ct_num, state);
}


/*--------------------------------------------------------------------------*/
/*! \brief Create a constraints manager structure.
 *
 *  \param nb_dof: The number of degree of freedom in the robot 
 *                 (or multi-robot)
 *
 *  \return The new constraints manager (NULL if there is a memory erreor).
 */
p3d_cntrt_management * p3d_create_cntrt_manager(int nb_dof)
{
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
  for(i=0; i<nb_dof; i++)
    { cntrt_manager->in_cntrt[i] = DOF_WITHOUT_CNTRT; }

  cntrt_manager->cntrt_call_list = dbl_list_pointer_init();

  return(cntrt_manager);
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy a constraints manager structure.
 *
 *  \param cntrt_manager: The constraints manager.
 */
void p3d_destroy_cntrt_manager(p3d_cntrt_management * cntrt_manager)
{
  int i;
  pp3d_cntrt ct;

  if (cntrt_manager != NULL) {

    // free of st_iksol
    if(st_iksol != NULL) {
      MY_FREE(st_iksol, int, cntrt_manager->ncntrts);
      MY_FREE(st_niksol, int, cntrt_manager->ncntrts);
    }

    for(i=(cntrt_manager->ncntrts)-1; i>=0; i--) {
      ct = cntrt_manager->cntrts[i];
      if(ct->enchained != NULL) 
	{ MY_FREE(ct->enchained, p3d_cntrt *, ct->nenchained); }
      if(ct->ct_to_update != NULL) 
	{ MY_FREE(ct->ct_to_update, p3d_cntrt *, ct->nctud); }
      if(ct->rlgPt != NULL) {
	p3d_destroy_rlg(ct->rlgPt);
      }
      if(ct->parallel_sys_data != NULL) {
	p3d_destroy_parallel_sys_data(ct->parallel_sys_data);
      }      
      if(ct->bio_ik_data != NULL) {
	MY_FREE(ct->bio_ik_data, bio_6R_ik_data, 1);
      }  
      MY_FREE(ct,p3d_cntrt,1);      
    }
    
    dbl_list_destroy(cntrt_manager->cntrt_call_list);  
    MY_FREE(cntrt_manager->cntrts,p3d_cntrt *,cntrt_manager->ncntrts);
    cntrt_manager->cntrts  = NULL;
    cntrt_manager->ncntrts = 0;
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
 */
void p3d_clear_cntrt_manager(p3d_cntrt_management * cntrt_manager)
{
  int i;
  pp3d_cntrt ct;

  if (cntrt_manager != NULL) {
    for(i=(cntrt_manager->ncntrts)-1; i>=0; i--) {
      ct = cntrt_manager->cntrts[i];
      if(ct->enchained != NULL) 
	{ MY_FREE(ct->enchained, p3d_cntrt *, ct->nenchained); }
      if(ct->rlgPt != NULL) {
	p3d_destroy_rlg(ct->rlgPt);
      }
      if(ct->parallel_sys_data != NULL) {
	p3d_destroy_parallel_sys_data(ct->parallel_sys_data);
      }      
      if(ct->bio_ik_data != NULL) {
	MY_FREE(ct->bio_ik_data, bio_6R_ik_data, 1);
      } 
      MY_FREE(ct,p3d_cntrt,1);
    }
    
    dbl_list_destroy(cntrt_manager->cntrt_call_list);  
    MY_FREE(cntrt_manager->cntrts,p3d_cntrt *,cntrt_manager->ncntrts);
    cntrt_manager->ncntrts = 0;
    cntrt_manager->cntrts  = NULL;
    for(i=0; i<cntrt_manager->nb_dof; i++)
      { cntrt_manager->in_cntrt[i] = DOF_WITHOUT_CNTRT; }
  }
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
				 p3d_cntrt_management * cntrt_manager_destPt)
{
  int j, test;
  p3d_cntrt * cntrtPt;

  if ((cntrt_manager_srcPt == NULL) || (cntrt_manager_destPt == NULL) ||
      (cntrt_manager_srcPt->nb_dof != cntrt_manager_destPt->nb_dof))
    { return FALSE; }
  p3d_clear_cntrt_manager(cntrt_manager_destPt);

  /* copy the constraints */
  test = TRUE;
  for(j=0; j<cntrt_manager_srcPt->ncntrts; j++) {
    cntrtPt = cntrt_manager_srcPt->cntrts[j];
    if(cntrtPt != NULL) {
      if (!p3d_create_constraint(cntrt_manager_destPt, cntrtPt->namecntrt,
				 cntrtPt->npasjnts, cntrtPt->pasjnts, 
				 cntrtPt->pas_jnt_dof, cntrtPt->pas_rob_dof,
				 cntrtPt->nactjnts, cntrtPt->actjnts,  
				 cntrtPt->act_jnt_dof, cntrtPt->act_rob_dof,
				 cntrtPt->ndval, cntrtPt->argu_d,
				 cntrtPt->nival, cntrtPt->argu_i,
				 -1, cntrtPt->active))
	{ test = FALSE; }
    }
  }
  return test;
}

/* ------------------------------------------------------------------ */

p3d_cntrt *p3d_get_current_cntrt()
{
 return last_cntrt_set;
}

int p3d_set_current_cntrt(p3d_cntrt *ct)
{
  last_cntrt_set = ct;
  return TRUE;
}

/* ------------------------------------------------------------------ */

/* function that fills the field Tatt of a cntrt  */
/* Tatt is the transformation between a joint and */
/* the frame asociated to the grasp               */
int p3d_set_cntrt_Tatt(int ct_num, double *matelem)
{
  p3d_rob *r;
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  return p3d_set_cntrt_Tatt_r(r,ct_num,matelem);
}

int p3d_set_cntrt_Tatt_r(p3d_rob *r, int ct_num, double *matelem)
{p3d_cntrt *ct;

 if((r->cntrt_manager->cntrts == NULL)||(r->cntrt_manager->cntrts[ct_num] == NULL))
   return(FALSE);
 
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

 return(TRUE);
}

/* ------------------------------------------------------------------ */

int p3d_update_all_jnts_state(int mode)
{p3d_rob *r;
 int i; 

 r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
 for(i=0;i<r->cntrt_manager->ncntrts;i++) {
   /* mode == 0 -> normal case      -> update all cntrts */
   /* mode == 1 -> from rw_scenario -> not need to update active cntrts */
   if((mode == 0) || !(r->cntrt_manager->cntrts[i]->active))
     { p3d_update_jnts_state(r->cntrt_manager->cntrts[i], r->cntrt_manager->cntrts[i]->active); }
 }

 return TRUE;
}
 

int p3d_update_jnts_state(p3d_cntrt *ct, int cntrt_state)
{
 int i,j,k;
 int before[MAX_ARGU_CNTRT];

 if(cntrt_state == 0) {
   for(i=0;i<ct->npasjnts;i++) {
     ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] = DOF_WITHOUT_CNTRT;
   }
   if(ct->enchained != NULL) {
     for(i=0; i<ct->nenchained; i++) {
       for(k=0; k<ct->enchained[i]->nactjnts; k++) {
	 ct->cntrt_manager->in_cntrt[ct->enchained[i]->act_rob_dof[k]] = 
	   DOF_ACTIF;
       }
     }
   }     
 }
 else {   /* cntrt_state == 1 */
   for(i=0;i<ct->npasjnts;i++) {
     if(ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] != DOF_PASSIF) {
       before[i] = ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[i]];
       ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[i]] = DOF_PASSIF;
     }
     else {
       for(j=0;j<i;j++) {
	 ct->cntrt_manager->in_cntrt[ct->pas_rob_dof[j]] = before[j]; 
       }
       PrintInfo(("ERROR : Joint %d is already passive\n", ct->pasjnts[i]->num));
       return(FALSE);
     }
   }
 }
 return(TRUE);
}

static int p3d_change_act_rob_dof_state(p3d_cntrt *ct, int rob_dof, int state)
{
  int i;

  for(i=0; i<ct->nactjnts; i++) {
    if(ct->act_rob_dof[i] == rob_dof) {
      ct->actjnt_state[i] = state;
      return(TRUE);
    }
  }
  return(FALSE);
}

/*! \brief Search if the constraints depend on the modified joint */
static int p3d_go_into_cntrt_fct(p3d_cntrt *ct)
{
  int i,modif;

  modif = 0;
  for(i=0; i<ct->nactjnts; i++) {
    if(p3d_jnt_get_dof_is_modified(ct->actjnts[i],ct->act_jnt_dof[i])) {
      if(ct->actjnt_state[i] == 0) 
	return FALSE;
      modif = 1;
    }
  }
  return (modif);
}

/* ------------------------------------------------------------------ */

void p3d_col_deactivate_cntrt_manager_pairs(p3d_cntrt_management *
					    cntrt_manager)
{
  p3d_cntrt *ct;

  if ((cntrt_manager != NULL) && (cntrt_manager->cntrts != NULL)) {
    ct = cntrt_manager->cntrts[0];
    while(ct != NULL) {
      if(ct->active)
	{ p3d_col_deactivate_one_cntrt_pairs(ct); }
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
void p3d_col_deactivate_cntrt_manager_pairs_into(
				 p3d_cntrt_management *	 cntrt_manager,
				 p3d_collision_pair * col_pairPt)
{
  p3d_cntrt *ct;
  
  if ((cntrt_manager != NULL) && (cntrt_manager->cntrts != NULL)) {
    ct = cntrt_manager->cntrts[0];
    while(ct != NULL) {
      if(ct->active)
	{ p3d_col_deactivate_one_cntrt_pairs_into(ct, col_pairPt); }
      ct = ct->next_cntrt;
    }
  }
}

void p3d_col_deactivate_cntrt_pairs(void)
{
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
					     p3d_collision_pair * col_pairPt)
{
  int i;
  
  for(i=0; (i<MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
    p3d_col_pair_deactivate_pair(col_pairPt, ct->col_pairs[0][i], 
				 ct->col_pairs[1][i]);
  }
  if(ct->enchained != NULL) {
    if(strcmp(ct->namecntrt,"p3d_planar_closed_chain")!=0) {
      for(i=0; i<ct->nenchained; i++) {
	p3d_col_deactivate_enchained_cntrts_pairs(ct, ct->enchained[i],
						  ct->enchained_rob_dof[i]);
      }
    }
  }
}

void p3d_col_deactivate_one_cntrt_pairs(p3d_cntrt *ct)
{
  int i;
  
  for(i=0; (i<MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
    p3d_col_deactivate_obj_obj(ct->col_pairs[0][i], ct->col_pairs[1][i]);
  }
  if(ct->enchained != NULL) {
    if(strcmp(ct->namecntrt,"p3d_planar_closed_chain")!=0) {
      for(i=0; i<ct->nenchained; i++) {
	p3d_col_deactivate_enchained_cntrts_pairs(ct, ct->enchained[i],
						  ct->enchained_rob_dof[i]);
      }
    }
  }
}

void p3d_col_activate_one_cntrt_pairs(p3d_cntrt *ct)
{
  int i;
  
  for(i=0; (i<MAX_ARGU_CNTRT) && (ct->col_pairs[0][i] != NULL); i++) {
    p3d_col_activate_obj_obj(ct->col_pairs[0][i], ct->col_pairs[1][i]);
  }
  if(ct->enchained != NULL) {
    if(strcmp(ct->namecntrt,"p3d_planar_closed_chain")!=0) {
      for(i=0; i<ct->nenchained; i++) {
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
void p3d_col_deactivate_enchained_cntrts_pairs_into(
				    p3d_cntrt *ct, p3d_cntrt *ect, int rob_dof,
				    p3d_collision_pair * col_pairPt)
{
  int i,j;

  if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
    for(i=0; i<ct->nactjnts; i++) {
      if(ct->act_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_pair_deactivate_pair(col_pairPt, ct->actjnts[i]->o,
					 ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_pair_deactivate_pair(col_pairPt, ct->actjnts[i]->o,
					 ect->actjnts[j]->o);
	  }
	}
      }
    }
    for(i=0; i<ct->npasjnts; i++) {
      if(ct->pas_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_pair_deactivate_pair(col_pairPt, ct->pasjnts[i]->o,
					 ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_pair_deactivate_pair(col_pairPt, ct->pasjnts[i]->o,
					 ect->actjnts[j]->o);
	  }
	}
      }
    }
  }
}


void p3d_col_deactivate_enchained_cntrts_pairs(p3d_cntrt *ct, p3d_cntrt *ect, 
					       int rob_dof)
{
  int i,j;

  if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
    for(i=0; i<ct->nactjnts; i++) {
      if(ct->act_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_deactivate_obj_obj(ct->actjnts[i]->o, ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_deactivate_obj_obj(ct->actjnts[i]->o, ect->actjnts[j]->o);
	  }
	}
      }
    }
    for(i=0; i<ct->npasjnts; i++) {
      if(ct->pas_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_deactivate_obj_obj(ct->pasjnts[i]->o, ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_deactivate_obj_obj(ct->pasjnts[i]->o, ect->actjnts[j]->o);
	  }
	}
      }
    }
  }
}
 
void p3d_col_activate_enchained_cntrts_pairs(p3d_cntrt *ct, p3d_cntrt *ect, 
					     int rob_dof)
{
  int i,j;

  if ((ct->col_pairs[0][0] != NULL) && (ect->col_pairs[0][0] != NULL)) {
    for(i=0; i<ct->nactjnts; i++) {
      if(ct->act_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_activate_obj_obj(ct->actjnts[i]->o, ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_activate_obj_obj(ct->actjnts[i]->o, ect->actjnts[j]->o);
	  }
	}
      }
    }
    for(i=0; i<ct->npasjnts; i++) {
      if(ct->pas_rob_dof[i] != rob_dof) {
	for(j=0; j<ect->npasjnts; j++) {
	  if(ect->pas_rob_dof[j] != rob_dof) {
	    p3d_col_activate_obj_obj(ct->pasjnts[i]->o, ect->pasjnts[j]->o);
	  }
	}
	for(j=0; j<ect->nactjnts; j++) {
	  if(ect->act_rob_dof[j] != rob_dof) {
	    p3d_col_activate_obj_obj(ct->pasjnts[i]->o, ect->actjnts[j]->o);
	  }
	}
      }
    }
  }
}


/* ------------------------------------------------------------------ */

static void p3d_add_to_cntrts_chain(p3d_cntrt *ct,p3d_cntrt *ect, int rob_dof)
{
  ct->enchained = MY_REALLOC(ct->enchained, pp3d_cntrt,
			     ct->nenchained, ct->nenchained+1);
  ct->enchained[ct->nenchained] = ect;
  ct->enchained_rob_dof[ct->nenchained] = rob_dof;
  ct->nenchained++;
}


void p3d_enchain_cntrt(p3d_cntrt *ect, int rob_dof, int arg)
{
  p3d_cntrt_management *cntrt_manager;
  p3d_cntrt *ct;
  int j, k;

  cntrt_manager = ect->cntrt_manager;
  for(k=0; k < cntrt_manager->ncntrts; k++) {
    ct = cntrt_manager->cntrts[k];
    if(arg == 1) {
      /* on cherche la premiere cntrt pour laquelle rob_dof est active */
      for(j=0; j<ct->nactjnts; j++) {
	if (rob_dof == ct->act_rob_dof[j]) {
	  p3d_add_to_cntrts_chain(ct, ect, rob_dof);
	  /* on sort ect de la liste primaire de cntrts du robot 
	     (si ct est active) */
	  if(ct->active) {
	    p3d_change_act_rob_dof_state(ect,rob_dof,0);
	    /*    ect->prev_cntrt->next_cntrt = NULL; */
	  }
	  return;
	}
      }
    } else if(arg == 2) {  
      /* on cherche la cntrt pour laquelle J est passive 
	 (il peut avoir que UNE) */
      for(j=0; j<ct->npasjnts; j++) {
	if(rob_dof == ct->pas_rob_dof[j]) {
	  p3d_add_to_cntrts_chain(ct, ect, rob_dof);
	  if(ct->active) {
	    p3d_change_act_rob_dof_state(ect,rob_dof,0);
	    /*    ect->prev_cntrt->next_cntrt = NULL; */
	  }
	  return;
	}
      }
    }
  }
}

void p3d_unchain_cntrts(p3d_cntrt *ct)
{int i;

 for(i=0; i<ct->nenchained; i++) {
   p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
 }
}

void p3d_reenchain_cntrts(p3d_cntrt *ct)
{int i;

 for(i=0; i<ct->nenchained; i++) {
   p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
 }
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
static p3d_cntrt * s_p3d_create_cntrts(p3d_cntrt_management * cntrt_manager)
{
  p3d_cntrt * ct, *cct;
  int i;

  ct = MY_ALLOC(p3d_cntrt, 1);
  if (ct == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  if(cntrt_manager->cntrts == NULL) {
    ct->prev_cntrt = NULL; 
    cntrt_manager->cntrts = MY_ALLOC(pp3d_cntrt, 1);
  } else {
    cntrt_manager->cntrts = MY_REALLOC(cntrt_manager->cntrts, pp3d_cntrt, 
				       cntrt_manager->ncntrts,
				       cntrt_manager->ncntrts+1);
    cct = cntrt_manager->cntrts[0];
    while(cct->next_cntrt != NULL)
      { cct = cct->next_cntrt; }
    cct->next_cntrt = ct;
    ct->prev_cntrt = cct; 
  }
  dbl_list_add_link(cntrt_manager->cntrt_call_list,ct);
  ct->next_cntrt = NULL;
  ct->num = cntrt_manager->ncntrts;
  cntrt_manager->ncntrts++;
  cntrt_manager->cntrts[ct->num] = ct;

  ct->namecntrt[0] = '\0'; /* No name */
  ct->active     = FALSE;
  ct->fct_cntrt  = NULL;
  ct->nactjnts   = 0;
  ct->npasjnts   = 0;
  ct->ndval      = 0;
  ct->nival      = 0;
  ct->nenchained = 0;
  ct->enchained  = NULL;
  ct->reshoot_ct = NULL; 
  ct->nctud      = 0; 
  ct->ct_to_update = NULL;
  ct->rlgPt      = NULL;
  ct->parallel_sys_data = NULL;
  ct->bio_ik_data = NULL;
  p3d_mat4Copy(p3d_mat4IDENTITY, ct->Tatt);
  ct->cntrt_manager = cntrt_manager;
  for(i=0; i<MAX_ARGU_CNTRT; i++) {    
    ct->actjnts[i]      = NULL;
    ct->act_jnt_dof[i]  = -1;
    ct->act_rob_dof[i]  = -1;
    ct->actjnt_state[i] =  1;
    ct->pasjnts[i]      = NULL;
    ct->pas_jnt_dof[i]  = -1;
    ct->pas_rob_dof[i]  = -1;
    ct->argu_i[i]       =  0;
    ct->argu_d[i]       =  0.0;
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
p3d_cntrt * p3d_create_generic_cntrts(
	       p3d_cntrt_management * cntrt_manager, const char *namecntrt, 
	       int nb_passif, p3d_jnt ** pas_jntPt,
	       int * pas_jnt_dof, int * pas_rob_dof,
	       int nb_actif, p3d_jnt ** act_jntPt,
	       int * act_jnt_dof, int * act_rob_dof)
{
  int i,j;
  p3d_cntrt *ct;
  
  for(i=0; i<nb_passif; i++) {
    if(cntrt_manager->in_cntrt[pas_rob_dof[i]] == DOF_PASSIF) {
      PrintWarning(("ERROR: p3d_create_generic_cntrts: rob_dof is already a passive joint\n"));
      return NULL;
    }
  }
  for(i=0; i<nb_passif; i++) {
    for(j=i+1; j<nb_passif; j++) {
      if (pas_rob_dof[i] == pas_rob_dof[j]) {
	PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));
	return NULL;
      }
    }
    for(j=0; j<nb_actif; j++) {
      if (pas_rob_dof[i] == act_rob_dof[j]) {
	PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));
	return NULL;
      }
    }
  }
  for(i=0; i<nb_actif; i++) {
    for(j=i+1; j<nb_actif; j++) {
      if (act_rob_dof[i] == act_rob_dof[j]) {
	PrintWarning(("ERROR: p3d_create_generic_cntrts: use the same degree of freedom two time\n"));
	return NULL;
      }
    }
  }
  ct = s_p3d_create_cntrts(cntrt_manager);
  if (ct == NULL)
    { return NULL; }
  ct->npasjnts = nb_passif;
  for(i=0; i<nb_passif; i++) {
    ct->pasjnts[i]     = pas_jntPt[i];
    ct->pas_jnt_dof[i] = pas_jnt_dof[i];
    ct->pas_rob_dof[i] = pas_rob_dof[i];
    cntrt_manager->in_cntrt[pas_rob_dof[i]] = DOF_PASSIF;
  }
  ct->nactjnts = nb_actif;
  for(i=0; i<nb_actif; i++) {
    ct->actjnts[i]     = act_jntPt[i];
    ct->act_jnt_dof[i] = act_jnt_dof[i];
    ct->act_rob_dof[i] = act_rob_dof[i];
  }
  strcpy(ct->namecntrt, namecntrt);
  ct->active = TRUE;

  return ct;
}


/* rob_dof = val */ 
static int p3d_set_fixed_dof(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     double val, int ct_num, int state)
{    
  p3d_cntrt * ct;

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_FIXED_NAME, 
				     1, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_fixed_jnt;
    ct->ndval = 1;
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = val;
  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* rob_dofA = k * rob_dofB + C */ 
static int p3d_set_lin_rel_dofs(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     double k,  double C, int ct_num, int state)
{
  p3d_cntrt *ct;
  int i;

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_LIN_REL_NAME, 
				     1, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_lin_rel_dofs;
    ct->ndval = 2;
    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }

    /* NOTE: the deactivation or not of the pair could be chosen by 
             the user !!! */
/*     ct->col_pairs[0][0] = A_jntPt->o; */
/*     ct->col_pairs[1][0] = B_jntPt->o; */
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = k;
  ct->argu_d[1] = C;

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* A = k1 * B * cos C + k2 * B * sin C + k3 * C * cos B + k4 * C * sin B */ 
static int p3d_set_rel_dofs(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     double k1, double k2, double k3, double k4, int ct_num, int state)
{
  p3d_cntrt * ct;
  int i;

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_REL_NAME, 
				     1, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     2, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_rel_dofs;
    ct->ndval = 4;
    for(i=0; i<2; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
  }
  else {
    ct = cntrt_manager->cntrts[ct_num];
  }
  
  ct->argu_d[0] = k1;
  ct->argu_d[1] = k2;
  ct->argu_d[2] = k3;
  ct->argu_d[3] = k4;
  
  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* chaine fermee du type RRPR linkage                                       */
/* JO : articulation de rotation du corp a controler                        */
/* JA : articulation de translation. Doit etre place dans le point de tour  */
/*      entre le corp a controler et le corp de longeur variable.           */
/*      C'est l'articulation d'entree du movement                           */
/* JC : articulation de rotation de la basse du corp de longeur variable    */
static int p3d_set_RRPRlnk(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int ct_num, int state)
{
  p3d_jnt * JO, *JA, *JC;
  p3d_cntrt *ct;
  p3d_vector3 posi_jnt;
  double xO,yO,zO,xA,yA,zA,xC,yC,zC;
  double l_OA,l_CO,l_CA;
  p3d_vector3 v1,v2,v1vectv2,senseOA,senseCA,vaxe;
  double tetamod,chimod;
  int i;

  JO = pas_jntPt[0];
  JC = pas_jntPt[1];
  JA = act_jntPt[0];
  if ((JA->type != P3D_TRANSLATE) || (JO->type != P3D_ROTATE) ||
      (JC->type != P3D_ROTATE)) {
    PrintWarning(("ERROR: p3d_set_RRPRlnk: wrong type of joint !!!\n"));
    return FALSE;
  }

  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_RRPR_NAME, 
				     2, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_RRPRlnk;
    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
 
    i = 0;
    ct->col_pairs[0][i] = JO->o;
    ct->col_pairs[1][i] = JA->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
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

  l_OA = sqrt(sqr(xA-xO)+sqr(yA-yO)+sqr(zA-zO));
  l_CO = sqrt(sqr(xC-xO)+sqr(yC-yO)+sqr(zC-zO));
  l_CA = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));

  ct->argu_d[0] = l_OA;
  ct->argu_d[1] = l_CO;
   
  v1[0] = xC-xO;
  v1[1] = yC-yO;
  v1[2] = zC-zO;
  v2[0] = xA-xO; 
  v2[1] = yA-yO;
  v2[2] = zA-zO;
  tetamod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/
			      (p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseOA);

  p3d_jnt_get_dof_cur_axis(JO, 0, v1vectv2);

  p3d_vectNormalize(v1vectv2,vaxe);
  p3d_vectSub(senseOA,vaxe,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001) || (fabs(v1vectv2[1])>0.0001) ||
     (fabs(v1vectv2[2])>0.0001)) {
    tetamod = 180.0 - tetamod;
    ct->argu_i[5] = 2;
  } else 
    { ct->argu_i[5] = 1; }

  v2[0] = xA-xC; 
  v2[1] = yA-yC;
  v2[2] = zA-zC;  
  chimod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2) /
			     (p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseCA);
  p3d_vectSub(senseOA,senseCA,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001) || (fabs(v1vectv2[1])>0.0001) ||
     (fabs(v1vectv2[2])>0.0001))
    { chimod = -chimod; }
  if(ct->argu_i[5] == 2) 
    { chimod = 180.0 - chimod; }

  ct->argu_d[7] = tetamod-p3d_jnt_get_dof_deg(JO, 0);
  ct->argu_d[8] = chimod-p3d_jnt_get_dof_deg(JC, 0);
  
  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}
  

/* chaine fermee du type 4R linkage                                          */
/* JO : articulation de rotation du corp d'entree de mouvement               */
/* JA : articulation de rotation entre le corp d'entree et le corps          */
/*      intermediaire                                                        */
/* JB : articulation de rotation entre le corp intermediaire et le corp de   */
/*      sortie. Placee dans le corp intermediaire                            */
/* JC : articulation de rotation du corp de sortie de mouvement              */
static int p3d_set_4Rlnk(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int ct_num, int state)
{
  p3d_jnt * JO, *JA, *JB, *JC;
  p3d_cntrt *ct;
  double tetamax,tetamin,calcint;
  p3d_vector3 posi_jnt;
  double xO,yO,zO,xA,yA,zA,xB,yB,zB,xC,yC,zC;
  p3d_matrix3 mat_pl;
  double l_OA,l_AB,l_BC,l_CO;
  p3d_vector3 v1,v2,v1vectv2,senseOA,senseCB,vaxe;
  double tetamod,chimod,fimod;
  int i;

  JO = act_jntPt[0];
  JA = pas_jntPt[0];
  JB = pas_jntPt[1];
  JC = pas_jntPt[2];
  if ((JA->type != P3D_ROTATE) || (JO->type != P3D_ROTATE) ||
      (JC->type != P3D_ROTATE) || (JB->type != P3D_ROTATE)) {
    PrintWarning(("ERROR: p3d_set_3RPRlnk: wrong type of joint !!!\n"));
    return FALSE;
  }
  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_4R_NAME, 
				     3, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_4Rlnk;
    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
    
    ct->col_pairs[0][0] = JA->o;
    ct->col_pairs[1][0] = JC->o;
  }
  else {
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

  mat_pl[0][0] = xA-xO;
  mat_pl[1][0] = xB-xO;
  mat_pl[2][0] = xC-xO;
  mat_pl[0][1] = yA-yO;
  mat_pl[1][1] = yB-yO;
  mat_pl[2][1] = yC-yO;
  mat_pl[0][2] = zA-zO;
  mat_pl[1][2] = zB-zO;
  mat_pl[2][2] = zC-zO;

  l_OA = sqrt(sqr(xA-xO)+sqr(yA-yO)+sqr(zA-zO));
  l_AB = sqrt(sqr(xA-xB)+sqr(yA-yB)+sqr(zA-zB));
  l_BC = sqrt(sqr(xC-xB)+sqr(yC-yB)+sqr(zC-zB));
  l_CO = sqrt(sqr(xC-xO)+sqr(yC-yO)+sqr(zC-zO));
  if((l_OA>(l_AB+l_BC+l_CO)) || (l_AB>(l_OA+l_BC+l_CO)) || 
     (l_BC>(l_AB+l_OA+l_CO)) || (l_CO>(l_AB+l_BC+l_OA))) {
    PrintWarning((("ERROR: p3d_close_4Rlnk: chain can not be closed : one of the links is too long\n")));
    return(FALSE);
  }

  ct->argu_d[0] = l_OA;
  ct->argu_d[1] = l_AB;
  ct->argu_d[2] = l_BC;
  ct->argu_d[3] = l_CO;
  
  calcint = (sqr(l_CO)+sqr(l_OA)-sqr(l_AB-l_BC))/(2*l_OA*l_CO);
  if((calcint>=1.0)||(calcint<=-1.0)) tetamin = 23.0;
  else tetamin = acos(calcint);
  calcint = (sqr(l_CO)+sqr(l_OA)-sqr(l_AB+l_BC))/(2*l_OA*l_CO);
  if((calcint>=1.0)||(calcint<=-1.0)) tetamax = 23.0;
  else tetamax = acos(calcint);
  
  ct->argu_d[4] = tetamin;   /* in radians !! */
  ct->argu_d[5] = tetamax;   /* if the value is 23.0 -> not limit */
 
  v1[0] = xC-xO; 
  v1[1] = yC-yO;
  v1[2] = zC-zO;
  v2[0] = xA-xO; 
  v2[1] = yA-yO;
  v2[2] = zA-zO;  
  tetamod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/
			      (p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseOA);

  p3d_jnt_get_dof_cur_axis(JO, 0, v1vectv2);

  p3d_vectNormalize(v1vectv2,vaxe);
  p3d_vectSub(senseOA,vaxe,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
    tetamod = 180.0 - tetamod;
    ct->argu_i[6] = 2;
  }
  else 
    ct->argu_i[6] = 1;

  v2[0] = xB-xC; 
  v2[1] = yB-yC;
  v2[2] = zB-zC;  
  chimod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseCB);
  p3d_vectSub(senseOA,senseCB,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) chimod = -chimod;
  if(ct->argu_i[6] == 2)  chimod = 180.0 - chimod;

  v1[0] = xA-xO; 
  v1[1] = yA-yO;
  v1[2] = zA-zO;
  v2[0] = xB-xA; 
  v2[1] = yB-yA;
  v2[2] = zB-zA;  
  if(ct->argu_i[6] == 2)
    fimod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  else
    fimod = -(180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));


  ct->argu_d[7] = tetamod-p3d_jnt_get_dof_deg(JO, 0);
  ct->argu_d[8] = chimod-p3d_jnt_get_dof_deg(JC, 0);
  ct->argu_d[9] = fimod-p3d_jnt_get_dof_deg(JA, 0);

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* << modif EF pour Delmia */
/* chaine fermee du type RRPR linkage mais controlee par JC (JO ?)          */
/* JO : articulation de rotation du corp a controler                        */
/*      c'est le joint de controle                                          */
/* JA : articulation de translation. Doit etre place dans le point de tour  */
/*      entre le corp a controler et le corp de longeur variable.           */
/* JC : articulation de rotation de la basse du corp de longeur variable    */
static int p3d_set_P3Rlnk(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int ct_num, int state)
{
  p3d_jnt * JO, *JA, *JC;
  p3d_cntrt *ct;
  p3d_vector3 posi_jnt;
  double xO,yO,zO,xA,yA,zA,xC,yC,zC;
  double l_OA,l_CO,l_CA;
  int i;

  JC = pas_jntPt[0];
  JA = pas_jntPt[1];
  JO = act_jntPt[0];
  if ((JA->type != P3D_TRANSLATE) || (JO->type != P3D_ROTATE) ||
      (JC->type != P3D_ROTATE)) {
    PrintWarning(("ERROR: p3d_set_RRPRlnk: wrong type of joint !!!\n"));
    return FALSE;
  }

  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_P3R_NAME, 
				     2, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_P3Rlnk;
    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
 
    i = 0;
    ct->col_pairs[0][i] = JC->o;
    ct->col_pairs[1][i] = JA->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
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

  l_OA = sqrt(sqr(xA-xO)+sqr(yA-yO)+sqr(zA-zO));
  l_CO = sqrt(sqr(xC-xO)+sqr(yC-yO)+sqr(zC-zO));
  l_CA = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));

  ct->argu_d[0] = l_OA;
  ct->argu_d[1] = l_CO;
  ct->argu_d[2] = l_CA;
  
  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* chaine fermee du type 3RPR linkage */
/* JO : articulation de rotation du corp d'entree de mouvement */
/* JA : articulation de rotation entre le corp d'entree et le corps */
/*      intermediaire */
/* JB : articulation de translation entre le corp intermediaire et le corps */
/*      de sortie. Placee dans le point qui serait la rotation avec le      */
/*      corps intermediaire */
/* JC : articulation de rotation du corp de sortie de mouvement */
static int p3d_set_3RPRlnk(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int ct_num, int state)
{
  p3d_jnt * JO, *JA, *JB, *JC;
  p3d_cntrt *ct;
  p3d_vector3 posi_jnt;
  double xO,yO,zO,xA,yA,zA,xB,yB,zB,xC,yC,zC;
  p3d_matrix3 mat_pl;
  double l_OA,l_AB,l_BC,l_CO,l_CA;
  p3d_vector3 v1,v2,v1vectv2,senseOA,senseCB,vaxe;
  double tetamod,chimod,fimod;
  int i;

  JO = act_jntPt[0];
  JA = pas_jntPt[0];
  JB = act_jntPt[1];
  JC = pas_jntPt[1];
  if ((JA->type != P3D_ROTATE) || (JO->type != P3D_ROTATE) ||
      (JC->type != P3D_ROTATE) || (JB->type != P3D_ROTATE)) {
    PrintWarning(("ERROR: p3d_set_3RPRlnk: wrong type of joint !!!\n"));
    return FALSE;
  }
  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_3RPR_NAME, 
				     2, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     2, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_3RPRlnk;
    for(i=0; i<2; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }

    i = 0;
    ct->col_pairs[0][i] = JA->o;
    ct->col_pairs[1][i] = JC->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
    ct->col_pairs[0][i] = JB->o;
    ct->col_pairs[1][i] = JO->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
    ct->col_pairs[0][i] = JA->o;
    ct->col_pairs[1][i] = JB->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
    ct->col_pairs[0][i] = JO->o;
    ct->col_pairs[1][i] = JC->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
    ct->col_pairs[0][i] = JO->prev_jnt->o;
    ct->col_pairs[1][i] = JA->o;
    if (ct->col_pairs[0][i]!=NULL) { i++; }
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

  mat_pl[0][0] = xA-xO;
  mat_pl[1][0] = xB-xO;
  mat_pl[2][0] = xC-xO;
  mat_pl[0][1] = yA-yO;
  mat_pl[1][1] = yB-yO;
  mat_pl[2][1] = yC-yO;
  mat_pl[0][2] = zA-zO;
  mat_pl[1][2] = zB-zO;
  mat_pl[2][2] = zC-zO;
  if(p3d_mat3Det(mat_pl) != 0.0) {
    PrintInfo((("ERROR: p3d_set_3RPRlnk: joints must be on the same planar\n")));
    return(FALSE);
  }

  l_OA = sqrt(sqr(xA-xO)+sqr(yA-yO)+sqr(zA-zO));
  l_AB = sqrt(sqr(xA-xB)+sqr(yA-yB)+sqr(zA-zB));
  l_BC = sqrt(sqr(xC-xB)+sqr(yC-yB)+sqr(zC-zB));
  l_CO = sqrt(sqr(xC-xO)+sqr(yC-yO)+sqr(zC-zO));
  l_CA = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));
  if((l_OA>(l_AB+l_BC+l_CO))||(l_AB>(l_OA+l_BC+l_CO))||(l_BC>(l_AB+l_OA+l_CO))||(l_CO>(l_AB+l_BC+l_OA))) {
    PrintInfo((("ERROR: p3d_set_3RPRlnk: chain can not be closed : one of the links is too long\n")));
    return(FALSE);
  }

  ct->argu_d[0] = l_OA;
  ct->argu_d[1] = l_AB;
  ct->argu_d[2] = l_CA;
  ct->argu_d[3] = l_CO;
  
  v1[0] = xC-xO; 
  v1[1] = yC-yO;
  v1[2] = zC-zO;
  v2[0] = xA-xO; 
  v2[1] = yA-yO;
  v2[2] = zA-zO;  
  tetamod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseOA);

  p3d_jnt_get_dof_cur_axis(JO, 0, vaxe);
  p3d_vectSub(senseOA,vaxe,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
    tetamod = 180.0 - tetamod;
    ct->argu_i[6] = 2;
  }
  else 
    ct->argu_i[6] = 1;

  v2[0] = xB-xC; 
  v2[1] = yB-yC;
  v2[2] = zB-zC;  
  chimod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  p3d_vectXprod(v1,v2,v1vectv2);
  p3d_vectNormalize(v1vectv2,senseCB);
  p3d_vectSub(senseOA,senseCB,v1vectv2);
  if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) chimod = -chimod;
  if(ct->argu_i[6] == 2)  chimod = 180.0 - chimod;
  v1[0] = xA-xO; 
  v1[1] = yA-yO;
  v1[2] = zA-zO;
  v2[0] = xB-xA; 
  v2[1] = yB-yA;
  v2[2] = zB-zA;  
  if(ct->argu_i[6] == 2)
    fimod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
  else
    fimod = -(180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));

  ct->argu_d[7] = tetamod-p3d_jnt_get_dof_deg(JO, 0);
  ct->argu_d[8] = chimod-p3d_jnt_get_dof_deg(JC, 0);
  ct->argu_d[9] = fimod-p3d_jnt_get_dof_deg(JA, 0);

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
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
static int p3d_set_jnt_on_ground(
	     p3d_cntrt_management * cntrt_manager,
	     int nb_pas, p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int nb_dval, double* Dval, int *Ival, int ct_num, int state)
{
  p3d_jnt *J;
  p3d_cntrt *ct;
  double dx,dy,dz,distance,tetamod,angcormod,angantmod;
  p3d_vector3 pos0_jnt,posi_jnt;
  int i, rot_axe = 0;
  p3d_vector3 v1,v2,v1vectv2,sense;

  J = pas_jntPt[0];
  if (nb_dval != nb_pas+4) {
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

  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_ON_GROUND_NAME, 
				     1, pas_jntPt,pas_jnt_dof, pas_rob_dof,
				     0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_jnt_on_ground;
    ct->ndval = 5;
    ct->nival = 3;

    if(pos0_jnt[0]==1.0)
      { rot_axe = 0; }
    else
      { rot_axe = 1; }
    ct->argu_i[3] = rot_axe;

    /* blocked joints */
    /* NOTE : this part must be out of this if when theated by the interface FORMconstraint */
    for(i = 1; i<nb_pas; i++) {
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
    
    distance = sqrt(sqr(dx)+sqr(dy)+sqr(dz));
    ct->argu_d[5] = distance;
    
    tetamod = (180.0/M_PI)*(J->v);
    ct->argu_d[2] = tetamod;
    
    v1[0] = dx; 
    v1[1] = dy;
    v1[2] = dz;
    v2[0] = 0.0; 
    v2[1] = 0.0;
    v2[2] = 1.0;  
    angcormod = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
    p3d_vectXprod(v1,v2,v1vectv2);
    p3d_vectNormalize(v1vectv2,sense);
    p3d_jnt_get_dof_cur_axis(J, 0, posi_jnt);
    p3d_vectSub(posi_jnt,sense,v1vectv2); 
    if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) angcormod = -angcormod;
    ct->argu_d[3] = angcormod;
    
    angantmod = (180.0/M_PI)*acos(J->prev_jnt->abs_pos[2][2]);
    if(rot_axe == 0) {if(J->prev_jnt->abs_pos[2][1] > 0.0) angantmod = -angantmod;}
    else {if(J->prev_jnt->abs_pos[2][0] < 0.0) angantmod = -angantmod;}
    ct->argu_d[4] = angantmod;
   
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = Dval[0];     /*z0 */

  ct->argu_d[1] = Dval[1];   /* ang_susp */

  ct->argu_i[0] = Ival[0];   /* turningsense */
  ct->argu_i[1] = Ival[1];   /* changesense */
  ct->argu_i[2] = Ival[2];   /* stay */

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* tour des roues dans une voiture traite comme hilare avec remorque                        */
/* JO : articulation qui serait le point d'atache de la remorque                            */
/* JR : articulation de la roue droite                                                      */
/* JL : articulation de la roue gauche                                                      */
/* h : distance entre le point d'atache et le centre de la transmision des roues derrireres */
/* a : distance entre les centres des roues d'avant                                         */
static int p3d_set_car_front_wheels(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     double h, double a, int ct_num, int state)
{
  p3d_jnt *JO, *JR, *JL;
  int i_dofJO;
  p3d_cntrt *ct;
  int i;

  JO = act_jntPt[0];
  i_dofJO = act_jnt_dof[0];
  JR = pas_jntPt[0];
  JL = pas_jntPt[1];

  if ((!p3d_jnt_is_dof_angular(JO, i_dofJO)) || (JR->type != P3D_ROTATE) ||
      (JL->type != P3D_ROTATE)) {
    PrintInfo(("ERROR: p3d_set_car_front_wheels: wrong type of joints\n"));
    return(FALSE);
  }

  if((h<0.0)||(a<0.0)) {
    PrintInfo(("ERROR: p3d_set_car_front_wheels: h and a must be positive\n"));
    return(FALSE);
  } 

  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager,CNTRT_CAR_FRONT_WHEELS_NAME,
				     2, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_car_front_wheels;
    ct->ndval = 2;

    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = h;
  ct->argu_d[1] = a;

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
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

static int p3d_set_cycab_wheels(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     double l1, double l2, double e, int ct_num, int state)
{
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

  if ((!p3d_jnt_is_dof_angular(JO, i_dofJO)) || 
      (J1->type != P3D_ROTATE) ||
      (J2->type != P3D_ROTATE) ||
      (J3->type != P3D_ROTATE) ||
      (J4->type != P3D_ROTATE)) {
    PrintInfo(("ERROR: p3d_set_cycab_wheels: wrong type of joints\n"));
    return(FALSE);
  }

  if((l1<0.0)||(l2<0.0)||(e<0.0)) {
    PrintInfo(("ERROR: p3d_set_cycab_wheels: l1, l2 and e must be positive\n"));
    return(FALSE);
  } 

  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager,CNTRT_CYCAB_WHEELS_NAME,
				     4, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_cycab_wheels;
    ct->ndval = 2;

    for(i=0; i<1; i++) {
      if(cntrt_manager->in_cntrt[act_rob_dof[i]] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, act_rob_dof[i], 
			  cntrt_manager->in_cntrt[act_rob_dof[i]]);
      } else 
	{ cntrt_manager->in_cntrt[act_rob_dof[i]] = DOF_ACTIF; }
    }
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = l1;
  ct->argu_d[1] = l2;
  ct->argu_d[2] = e;

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

/* chaine cinematique planar */
/* JE1 et JE2 sont les points a fermer */
static int p3d_set_planar_closed_chain(
	     p3d_cntrt_management * cntrt_manager,
	     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
	     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
	     int whatcase, int ct_num, int state)
{ 
  p3d_jnt * JE1, *JE2, *JB1, *JP1, *JP2;
  p3d_cntrt *ct;
  p3d_vector3 posi_jnt;
/*   int JP1,JP2; */
  double xE2,yE2,zE2,xE1,yE1,zE1,xP1,yP1,zP1,xP2,yP2,zP2;
  double xref,yref,zref;
  double l_E1P2,l_P2P1,l_E2P1;
  p3d_vector3 v1,v2,v1vectv2,sense,vaxe;
  double angmod;
  int i,j;

  JE1 = pas_jntPt[0];
  JE2 = pas_jntPt[1];
  JP1 = pas_jntPt[2];
  JP2 = pas_jntPt[3];
  JB1 = act_jntPt[0];

  if ((JE1->type != P3D_ROTATE) || (JE2->type != P3D_ROTATE) ||
      (JB1->type != P3D_ROTATE) || (JP1->type != P3D_ROTATE) ||
      (JP2->type != P3D_ROTATE) || (JE2->prev_jnt->rob != JE2->rob)) {
    PrintWarning(("ERROR: p3d_set_planar_closed_chain: wrong type of joint !!!\n"));
    return FALSE;
  }

  xref = yref = zref =0.0;


  p3d_jnt_get_cur_vect_point(JE2, posi_jnt); 
  xE2 = posi_jnt[0];
  yE2 = posi_jnt[1];
  zE2 = posi_jnt[2];
  p3d_jnt_get_cur_vect_point(JE1,posi_jnt); 
  xE1 = posi_jnt[0];
  yE1 = posi_jnt[1];
  zE1 = posi_jnt[2];
  p3d_jnt_get_cur_vect_point(JP2,posi_jnt); 
  xP2 = posi_jnt[0];
  yP2 = posi_jnt[1];
  zP2 = posi_jnt[2];
  p3d_jnt_get_cur_vect_point(JP1,posi_jnt); 
  xP1 = posi_jnt[0];
  yP1 = posi_jnt[1];
  zP1 = posi_jnt[2];
  
  l_E1P2 = sqrt(sqr(xE1-xP2)+sqr(yE1-yP2)+sqr(zE1-zP2));
  l_P2P1 = sqrt(sqr(xP2-xP1)+sqr(yP2-yP1)+sqr(zP2-zP1));
  l_E2P1 = sqrt(sqr(xE2-xP1)+sqr(yE2-yP1)+sqr(zE2-zP1));

  if(l_E2P1 > l_E1P2+l_P2P1){
    PrintInfo(("ERROR: p3d_set_planar_closed_chain: chain must be closable when defined\n"));
    return(FALSE);
  } 


  if(ct_num == -1) {
    ct = p3d_create_generic_cntrts(cntrt_manager, 
				     CNTRT_PLANAR_CLOSED_CHAIN_NAME, 
				     4, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_planar_closed_chain;
/*     ct->npasjnts = 4; */
    ct->nival = 1;
    
    ct->col_pairs[0][0] = JE1->prev_jnt->o;
    ct->col_pairs[1][0] = JE2->prev_jnt->o;
    
    /* different treatment for enchaining */    /* ESTO FUNCIONA SIEMPRE ????? */
    i = pas_rob_dof[1] - pas_jnt_dof[1] - pas_jntPt[1]->index_dof + 
      pas_jntPt[1]->prev_jnt->index_dof;
    for(j=0; j<pas_jntPt[1]->prev_jnt->dof_equiv_nbr; j++) {
      if(cntrt_manager->in_cntrt[i+j] != DOF_WITHOUT_CNTRT) {
	p3d_enchain_cntrt(ct, i+j, cntrt_manager->in_cntrt[i+j]);
      }
    }

    /* Parameters for JP1, JP2, and JE2 */
    /* lengths */
    ct->argu_d[0] = l_P2P1;
    ct->argu_d[1] = l_E1P2;

    /* reference angles */
    v1[0] = xref-xP1;
    v1[1] = yref-yP1;
    v1[2] = zref-zP1;
    v2[0] = xP2-xP1; 
    v2[1] = yP2-yP1;
    v2[2] = zP2-zP1;  
    angmod = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
    if(angmod == 0.0) {
      p3d_vectNormalize(v1,sense);
      p3d_vectNormalize(v2,vaxe);
      p3d_vectSub(sense,vaxe,v1vectv2);
      if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
	angmod = M_PI;
      }
    }
    else {
      p3d_vectXprod(v1,v2,v1vectv2);
      p3d_vectNormalize(v1vectv2,sense);

      p3d_jnt_get_dof_cur_axis(JP1, 0, vaxe);
      p3d_vectSub(sense,vaxe,v1vectv2);
      if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
	angmod = - angmod;
      }
    }
    ct->argu_d[3] = (180.0/M_PI)*(angmod - p3d_jnt_get_dof(JP1, 0));  /* stored in degrees */
    
    v1[0] = -v2[0];
    v1[1] = -v2[1];
    v1[2] = -v2[2];
    v2[0] = xE1-xP2; 
    v2[1] = yE1-yP2;
    v2[2] = zE1-zP2;  
    angmod = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
    if(angmod == 0.0) {
      p3d_vectNormalize(v1,sense);
      p3d_vectNormalize(v2,vaxe);
      p3d_vectSub(sense,vaxe,v1vectv2);
      if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
	angmod = M_PI;
      }
    }
    else {
      p3d_vectXprod(v1,v2,v1vectv2);
      p3d_vectNormalize(v1vectv2,sense);

      p3d_jnt_get_dof_cur_axis(JP1, 0, vaxe);
      p3d_vectSub(sense,vaxe,v1vectv2);
      if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) {
	angmod = - angmod;
      }
    }
    ct->argu_d[4] = (180.0/M_PI)*(angmod - p3d_jnt_get_dof(JP2, 0));  /* stored in degrees */

    /* reference for angle of JP3(==JE2) is not calculated */
  }

  else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_i[0] = whatcase;

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* fonctions pour traiter les contraintes                             */


static int p3d_fct_fixed_jnt(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
/*   p3d_jnt * jntPt; */
  double min,max;

/*   jntPt = ct->pasjnts[0]; */
/*   while ((jntPt!=NULL) && (jntPt != jntPt->rob->j_modif)) */
/*     { jntPt = jntPt->prev_jnt; } */
/*   if(jntPt!=NULL) { */
    p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], &min, &max);
    if(ct->argu_d[0]>max) {
      return(FALSE);
    }
    if(ct->argu_d[0]<min) {
      return(FALSE);
    }

    p3d_jnt_set_dof_deg(ct->pasjnts[0], ct->pas_jnt_dof[0], ct->argu_d[0]);
/*   } */
  return(TRUE);
}

static int p3d_fct_lin_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{ 
  double valso,valfo;
  double min,max;
  int i, I_can;
  double lastvalfo;

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
  if(p3d_go_into_cntrt_fct(ct)) {
    valso = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
    valfo = valso*ct->argu_d[0] + ct->argu_d[1]; 
    p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
    if(valfo>max) {
      return(FALSE);
    }
    if(valfo<min) {
      return(FALSE);
    }
    
    lastvalfo = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
    p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], valfo);
       
    if(ct->enchained != NULL) {
      I_can = 1;
      for(i=0; I_can && (i<ct->nenchained); i++) {
	p3d_change_act_rob_dof_state(ct->enchained[i],
				     ct->enchained_rob_dof[i],1);
	if(ct->enchained[i]->active)
	  // multiple iksol is not compatible (yet) with enchained !
	  I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
	p3d_change_act_rob_dof_state(ct->enchained[i],
				     ct->enchained_rob_dof[i],0);
      }
      if(!I_can) {
	p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalfo);
	return(FALSE);
      }
    } 
  }
  return(TRUE);
}

static int p3d_fct_rel_dofs(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 double min,max;
 double valJA,valJB,valJC;
 int i, I_can;
 double lastvalJA;

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {
   valJB = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
   valJC = p3d_jnt_get_dof_deg(ct->actjnts[1], ct->act_jnt_dof[1]);
   valJA = ct->argu_d[0] * valJB * cos((M_PI/180.0)*valJC) + ct->argu_d[1] * valJB * sin((M_PI/180.0)*valJC) + ct->argu_d[2] * valJC * cos((M_PI/180.0)*valJB) + ct->argu_d[3] * valJC * sin((M_PI/180.0)*valJB);
/*    valJA = ct->argu_d[0] * (valJB * cos((M_PI/180.0)*valJC) + valJB); */
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if(valJA>max) {
     return(FALSE);
   }
   if(valJA<min) {
     return(FALSE);
   }

   lastvalJA = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], valJA);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	  // multiple iksol is not compatible (yet) with enchained !
	  I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalJA);
       return(FALSE);
     }
   } 
 }
 return(TRUE);
}


static int p3d_fct_RRPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 double l_CA;
 double min,max;
 double teta,chi;
 p3d_vector3 posi_jnt;
 double xA,yA,zA,xC,yC,zC;
 int i, I_can;
 double lastvalteta,lastvalchi;

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {
   
   p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);

   p3d_jnt_get_cur_vect_point(ct->actjnts[0],posi_jnt); 
   xA = posi_jnt[0];
   yA = posi_jnt[1];
   zA = posi_jnt[2];

   p3d_jnt_get_cur_vect_point(ct->pasjnts[1],posi_jnt); 
   xC = posi_jnt[0];
   yC = posi_jnt[1];
   zC = posi_jnt[2];
   l_CA = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));

   if(l_CA>(ct->argu_d[0]+ct->argu_d[1])) {
     return(FALSE);
   }   
   if(l_CA<(fabs(ct->argu_d[0]-ct->argu_d[1]))) {
     return(FALSE);
   }   
   
   if(ct->argu_i[5] == 1) {    /* CASE A */
     teta = (180.0/M_PI)*acos((sqr(ct->argu_d[0])+sqr(ct->argu_d[1])-sqr(l_CA))/(2*(ct->argu_d[0])*(ct->argu_d[1])));
     chi = 180.0-(180.0/M_PI)*atan2((ct->argu_d[0])*sin((M_PI/180.0)*teta),((ct->argu_d[1])-(ct->argu_d[0])*cos((M_PI/180.0)*teta)));
   }
   else {                      /* CASE B */
     teta = -(180.0/M_PI)*acos((sqr(ct->argu_d[0])+sqr(ct->argu_d[1])-sqr(l_CA))/(2*(ct->argu_d[0])*(ct->argu_d[1])))+180.0;
     chi = (180.0/M_PI)*atan2((ct->argu_d[0])*sin((M_PI/180.0)*teta),((ct->argu_d[0])*cos((M_PI/180.0)*teta)+(ct->argu_d[1])));
   }

   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((teta - ct->argu_d[7] > max)||(teta - ct->argu_d[7] < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1],ct->pas_jnt_dof[1],&min,&max);
   if((chi - ct->argu_d[8] > max)||(chi - ct->argu_d[8] < min)) return(FALSE);


   lastvalteta = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1]);
   p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], teta - ct->argu_d[7]);
   p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], chi - ct->argu_d[8]);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	  // multiple iksol is not compatible (yet) with enchained !
	  I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalteta);
       p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], lastvalchi);
      return(FALSE);
     }
   }
 }
 return(TRUE);
}


static int p3d_fct_4Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 double min,max;
 double teta,tetamax,tetamin,fi,chi = 0.0;
 double a_te,b_te,c_te;
 int i, I_can;
 double lastvalteta,lastvalchi,lastvalfi;

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {
 
   if(ct->argu_i[6] == 1) {    /* CASE A */
     teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
     teta += ct->argu_d[7]; 
     tetamin = (180.0/M_PI)*ct->argu_d[4]+0.1;
     tetamax = (180.0/M_PI)*ct->argu_d[5];
     if(ct->argu_d[5] != 23.0) {
       if(teta>tetamax) {
	 return(FALSE);
       }
       else if((-teta)>tetamax) {
	 return(FALSE);
       }
     }
     if(ct->argu_d[4] != 23.0) {
       if(teta<tetamin) {
	 return(FALSE);
       }
       else if((360.0-teta)<tetamin) {
	 return(FALSE);
       }
     }
     
     if(teta>=360.0) {
       teta = 359.9;
     }
     if(teta<=-360.0) {
       teta = -359.9;
     }
         
     a_te = 2*(ct->argu_d[0])*(ct->argu_d[2])*cos((M_PI/180.0)*teta)-2*(ct->argu_d[2])*(ct->argu_d[3]);
     b_te = 2*(ct->argu_d[0])*(ct->argu_d[2])*sin((M_PI/180.0)*teta);
     c_te = sqr(ct->argu_d[3])+sqr(ct->argu_d[2])+sqr(ct->argu_d[0])-sqr(ct->argu_d[1])-2*(ct->argu_d[0])*(ct->argu_d[3])*cos((M_PI/180.0)*teta);
     
     if((b_te==0.0)&&(a_te==0.0)) {
	 if(teta==0.0) chi = 0.0;
	 else chi = 180.0;
     } 
     else {
       chi = (180.0/M_PI)*(atan(b_te/a_te)-acos(c_te/sqrt(sqr(a_te)+sqr(b_te))))+180.0;
     }

     fi = (180.0/M_PI)*atan2(((ct->argu_d[2])*sin((M_PI/180.0)*chi)-(ct->argu_d[0])*sin((M_PI/180.0)*teta)),((ct->argu_d[3])+(ct->argu_d[2])*cos((M_PI/180.0)*chi)-(ct->argu_d[0])*cos((M_PI/180.0)*teta)))-teta;
   }
   else {                      /* CASE B */  
     teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
     teta += ct->argu_d[7]; 
   
     tetamax = 180.0-(180.0/M_PI)*ct->argu_d[4]-0.1;
     tetamin = 180.0-(180.0/M_PI)*ct->argu_d[5];
     if(ct->argu_d[4] != 23.0) {
       if(teta>tetamax) {
	 return(FALSE);
       }
       else if((-teta)>tetamax) {
	 return(FALSE);
       }
     }
     if(ct->argu_d[5] != 23.0) {
       if(teta<tetamin) {
	 return(FALSE);
       }
       else if((360.0-teta)<tetamin) {
	 return(FALSE);
       }
     }
     
     if(teta>=360.0) {
       teta = 359.9;
     }
     if(teta<=-360.0) {
       teta = -359.9;
     }
        
     a_te = 2*(ct->argu_d[0])*(ct->argu_d[2])*cos((M_PI/180.0)*(180.0-teta))-2*(ct->argu_d[2])*(ct->argu_d[3]);
     b_te = 2*(ct->argu_d[0])*(ct->argu_d[2])*sin((M_PI/180.0)*(180.0-teta));
     c_te = sqr(ct->argu_d[3])+sqr(ct->argu_d[2])+sqr(ct->argu_d[0])-sqr(ct->argu_d[1])-2*(ct->argu_d[0])*(ct->argu_d[3])*cos((M_PI/180.0)*(180.0-teta));

     if((b_te==0.0)&&(a_te==0.0)) {
       if(teta==0.0) chi = 0.0;
       else if(teta==360.0) chi = 180.0;
     } 
     else
       chi = (180.0/M_PI)*-(atan2(b_te,a_te)-acos(c_te/sqrt(sqr(a_te)+sqr(b_te))))+180.0;

     fi = (180.0/M_PI)*atan2(((ct->argu_d[2])*sin((M_PI/180.0)*chi)-(ct->argu_d[0])*sin((M_PI/180.0)*teta)),(-(ct->argu_d[3])+(ct->argu_d[2])*cos((M_PI/180.0)*chi)-(ct->argu_d[0])*cos((M_PI/180.0)*teta)))-teta;
     if(fi < 0.0) fi += 360;
   }

   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[2],ct->pas_jnt_dof[2],&min,&max);
   if((chi - ct->argu_d[8] > max)||(chi - ct->argu_d[8] < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((fi - ct->argu_d[9] > max)||(fi - ct->argu_d[9] < min)) return(FALSE);

   lastvalteta = p3d_jnt_get_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0]);
   lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[2],ct->pas_jnt_dof[2]);
   lastvalfi = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   p3d_jnt_set_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0], teta - ct->argu_d[7]);
   p3d_jnt_set_dof_deg(ct->pasjnts[2],ct->pas_jnt_dof[2], chi - ct->argu_d[8]);
   p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], fi - ct->argu_d[9]);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	 // multiple iksol is not compatible (yet) with enchained !
	 I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0], lastvalteta);
       p3d_jnt_set_dof_deg(ct->pasjnts[2],ct->pas_jnt_dof[2], lastvalchi);
       p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalfi);
      return(FALSE);
     }
   } 
 }
 return(TRUE);
}

/* << modif EF pour Delmia */
static int p3d_fct_P3Rlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 double l_AO,l_OC,l_AC,dx;
 double min,max;
 double theta,phi;
 p3d_vector3 posi_jnt;
 p3d_matrix4 inv_pos0_O, rel_pos_OA, pos_A_in_O;
 double xA,yA,zA,xC,yC,zC;
 int i, I_can;
 double lastvaldx,lastvalphi;

 if(p3d_go_into_cntrt_fct(ct)) {
   
   p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);

   p3d_matInvertXform(ct->actjnts[0]->pos0, inv_pos0_O);
   p3d_matMultXform(inv_pos0_O, ct->pasjnts[1]->pos0, rel_pos_OA);
   /* Position of A needed */
   p3d_matMultXform(ct->actjnts[0]->abs_pos, rel_pos_OA, pos_A_in_O);
   xA = pos_A_in_O[0][3];
   yA = pos_A_in_O[1][3];
   zA = pos_A_in_O[2][3];
   
   p3d_jnt_get_cur_vect_point(ct->pasjnts[0],posi_jnt); 
   xC = posi_jnt[0];
   yC = posi_jnt[1];
   zC = posi_jnt[2];
   
   l_AC = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));
   
   dx = (l_AC - ct->argu_d[2]);
   p3d_jnt_get_dof_bounds(ct->pasjnts[1],ct->pas_jnt_dof[1],&min,&max);
   if((dx > max)||(dx < min))
     return(FALSE);

   l_OC = ct->argu_d[1];
   l_AO = ct->argu_d[0];
   theta = p3d_jnt_get_dof(ct->actjnts[0],ct->act_jnt_dof[0]);
   phi = -atan2(l_OC*sin(theta),l_OC*cos(theta) - l_AO) + theta + M_PI;
   phi = angle_limit_PI(phi);
   p3d_jnt_get_dof_bounds(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((phi > max)||(phi < min))
     return(FALSE);

   lastvalphi = p3d_jnt_get_dof(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   lastvaldx = p3d_jnt_get_dof(ct->pasjnts[1],ct->pas_jnt_dof[1]);
   p3d_jnt_set_dof(ct->pasjnts[0],ct->pas_jnt_dof[0], phi);
   p3d_jnt_set_dof(ct->pasjnts[1],ct->pas_jnt_dof[1], dx);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	 // multiple iksol is not compatible (yet) with enchained !
	 I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalphi);
       p3d_jnt_set_dof(ct->pasjnts[1],ct->pas_jnt_dof[1], lastvaldx);
      return(FALSE);
     }
   }
 }
 return(TRUE);
}

static int p3d_fct_3RPRlnk(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 double min,max;
 double teta,tetamax,tetamin,fi,chi = 0.0;
 double a_te,b_te,c_te;
 double xA,yA,zA,xB,yB,zB,xC,yC,zC;
 double l_BC,l_AC;
 p3d_jnt * JO, *JA, *JB, *JC;
 p3d_vector3 posi_jnt;
 double tetamax_r,tetamin_r,calcint;
 int i, I_can;
 double lastvalteta,lastvalchi,lastvalfi;

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {

   JO = ct->actjnts[0];
   JA = ct->pasjnts[0];
   JB = ct->actjnts[1];
   JC = ct->pasjnts[1];

/*    update_one_jnt_pos(r->joints[JO]); */
   p3d_update_this_robot_pos_without_cntrt_and_obj(JO->rob);

   p3d_jnt_get_cur_vect_point(JA,posi_jnt); 
   xA = posi_jnt[0];
   yA = posi_jnt[1];
   zA = posi_jnt[2];
   p3d_jnt_get_cur_vect_point(JB,posi_jnt); 
   xB = posi_jnt[0];
   yB = posi_jnt[1];
   zB = posi_jnt[2];
   p3d_jnt_get_cur_vect_point(JC,posi_jnt); 
   xC = posi_jnt[0];
   yC = posi_jnt[1];
   zC = posi_jnt[2];
   
   l_BC = sqrt(sqr(xC-xB)+sqr(yC-yB)+sqr(zC-zB));
   l_AC = sqrt(sqr(xC-xA)+sqr(yC-yA)+sqr(zC-zA));
   
   if(l_BC>(ct->argu_d[2]+ct->argu_d[1])) {
     return(FALSE);
  }
   else if(l_BC<(l_AC - ct->argu_d[1])) {
     return(FALSE);
   }   
   
   calcint = (sqr(ct->argu_d[3])+sqr(ct->argu_d[0])-sqr(ct->argu_d[1]-l_BC))/(2*ct->argu_d[0]*ct->argu_d[3]);
   if((calcint>=1.0)||(calcint<=-1.0)) tetamin_r = 23.0;
   else tetamin_r = acos(calcint);
   calcint = (sqr(ct->argu_d[3])+sqr(ct->argu_d[0])-sqr(ct->argu_d[1]+l_BC))/(2*ct->argu_d[0]*ct->argu_d[3]);
   if((calcint>=1.0)||(calcint<=-1.0)) tetamax_r = 23.0;
   else tetamax_r = acos(calcint);
   
   if(ct->argu_i[6] == 1) {    /* CASE A */
     teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
     teta += ct->argu_d[7]; 

     tetamin = (180.0/M_PI)*tetamin_r+0.1;
     tetamax = (180.0/M_PI)*tetamax_r;
     if(tetamax_r != 23.0) {
       if(teta>tetamax) {
	 return(FALSE);
       }
       else if((-teta)>tetamax) {
	 return(FALSE);
       }
     }
     if(tetamin_r != 23.0) {
       if(teta<tetamin) {
	 return(FALSE);
       }
       else if((360.0-teta)<tetamin) {
	 return(FALSE);
       }
     }
     
     if(teta>=360.0) {
       teta = 359.9;
     }
     if(teta<=-360.0) {
       teta = -359.9;
     }
         
     a_te = 2*(ct->argu_d[0])*(l_BC)*cos((M_PI/180.0)*teta)-2*(l_BC)*(ct->argu_d[3]);
     b_te = 2*(ct->argu_d[0])*(l_BC)*sin((M_PI/180.0)*teta);
     c_te = sqr(ct->argu_d[3])+sqr(l_BC)+sqr(ct->argu_d[0])-sqr(ct->argu_d[1])-2*(ct->argu_d[0])*(ct->argu_d[3])*cos((M_PI/180.0)*teta);
        
     if((b_te==0.0)&&(a_te==0.0)) {
       if(teta==0.0) chi = 0.0;
       else chi = 180.0;
     } 
     else {
       chi = (180.0/M_PI)*(atan(b_te/a_te)-acos(c_te/sqrt(sqr(a_te)+sqr(b_te))))+180.0;
     }
 
     fi = (180.0/M_PI)*atan2(((l_BC)*sin((M_PI/180.0)*chi)-(ct->argu_d[0])*sin((M_PI/180.0)*teta)),((ct->argu_d[3])+(l_BC)*cos((M_PI/180.0)*chi)-(ct->argu_d[0])*cos((M_PI/180.0)*teta)))-teta;
   }
   else {                      /* CASE B */  
     teta = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
     teta += ct->argu_d[7]; 
     
     tetamax = 180.0-(180.0/M_PI)*tetamin_r-0.1;
     tetamin = 180.0-(180.0/M_PI)*tetamax_r;
     if(tetamin_r != 23.0) {
       if(teta>tetamax) {
	 return(FALSE);
       }
       else if((-teta)>tetamax) {
	 return(FALSE);
       }
     }
     if(tetamax_r != 23.0) {
       if(teta<tetamin) {
	 return(FALSE);
       }
       else if((360.0-teta)<tetamin) {
	 return(FALSE);
       }
     }
     
     if(teta>=360.0) {
       teta = 359.9;
     }
     if(teta<=-360.0) {
       teta = -359.9;
     }
     
     a_te = 2*(ct->argu_d[0])*(l_BC)*cos((M_PI/180.0)*(180.0-teta))-2*(l_BC)*(ct->argu_d[3]);
     b_te = 2*(ct->argu_d[0])*(l_BC)*sin((M_PI/180.0)*(180.0-teta));
     c_te = sqr(ct->argu_d[3])+sqr(l_BC)+sqr(ct->argu_d[0])-sqr(ct->argu_d[1])-2*(ct->argu_d[0])*(ct->argu_d[3])*cos((M_PI/180.0)*(180.0-teta));
     
     if((b_te==0.0)&&(a_te==0.0)) {
       if(teta==0.0) chi = 0.0;
       else if(teta==360.0) chi = 180.0;
     } 
     else
       chi = (180.0/M_PI)*-(atan2(b_te,a_te)-acos(c_te/sqrt(sqr(a_te)+sqr(b_te))))+180.0;

     fi = (180.0/M_PI)*atan2(((l_BC)*sin((M_PI/180.0)*chi)-(ct->argu_d[0])*sin((M_PI/180.0)*teta)),(-(ct->argu_d[3])+(l_BC)*cos((M_PI/180.0)*chi)-(ct->argu_d[0])*cos((M_PI/180.0)*teta)))-teta;
     if(fi < 0.0) fi += 360;
   }

   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1],ct->pas_jnt_dof[1],&min,&max);
   if((chi - ct->argu_d[8] > max)||(chi - ct->argu_d[8] < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((fi - ct->argu_d[9] > max)||(fi - ct->argu_d[9] < min)) return(FALSE);

   lastvalteta = p3d_jnt_get_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0]);
   lastvalchi = p3d_jnt_get_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1]);
   lastvalfi = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   p3d_jnt_set_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0], teta - ct->argu_d[7]);
   p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], chi - ct->argu_d[8]);
   p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], fi - ct->argu_d[9]);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	 // multiple iksol is not compatible (yet) with enchained !
	 I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(ct->actjnts[0],ct->act_jnt_dof[0], lastvalteta);
       p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], lastvalchi);
       p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalfi);
      return(FALSE);
     }
   } 
 }
 return(TRUE);
}


static int p3d_fct_jnt_on_ground(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  double ang,teta_ant,value;
  p3d_vector4 posi_jnt;
  double xJ,yJ,zJ,xJA,yJA,zJA;
  static int f_t=1,sens;
  pp3d_jnt J, JA;
  int i;

  J = ct->pasjnts[0];
  JA = ct->pasjnts[0]->prev_jnt;

/*   if(r->joints[ct->pasjnts[0]]->pos[2][ct->argu_i[3]] == 0.0) { */

  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob);
  
    p3d_jnt_get_cur_vect_point(J,posi_jnt); 
    xJ = posi_jnt[0];
    yJ = posi_jnt[1];
    zJ = posi_jnt[2];
   
    teta_ant = (180.0/M_PI)*acos(JA->abs_pos[2][2]);
    if(ct->argu_i[3] == 0) {if(JA->abs_pos[2][1] > 0.0) teta_ant = -teta_ant;}
    else {if(JA->abs_pos[2][0] < 0.0) teta_ant = -teta_ant;}

    if(zJ > (ct->argu_d[5]+ct->argu_d[0])) {
      if(ct->argu_i[2] == 1)
	return(FALSE);
      else
	ang = ct->argu_d[1];
    }
    else { 
      ang = (180.0/M_PI)*acos((zJ - ct->argu_d[0])/ct->argu_d[5]);
      /* blocked joints */
      i = 1;
      while(ct->pasjnts[i] != NULL) {
	p3d_jnt_set_dof_deg(ct->pasjnts[i],ct->pas_jnt_dof[i], ct->argu_d[i+5]);
	i++;
      }
    }

    if((ct->argu_i[1] == 1)&&(f_t||(zJ >= (ct->argu_d[5]+ct->argu_d[0])))) {
      p3d_jnt_get_cur_vect_point(JA,posi_jnt); 
      xJA = posi_jnt[0];
      yJA = posi_jnt[1];
      zJA = posi_jnt[2];
      if((xJ>xJA)||(yJ>yJA)) sens = 0;
      else sens = 1;
      f_t = 0;
    }
    
    if(ct->argu_i[1] == 0) {
      if(ct->argu_i[0] == 0) { 
	value = 180.0 - ang + teta_ant + ct->argu_d[2] + ct->argu_d[3] - ct->argu_d[4];
      }
      else {
	value = - 180.0 + ang + teta_ant + ct->argu_d[2] + ct->argu_d[3] - ct->argu_d[4];
      }
    }
    else {
      if(sens) {
	value = 180.0 - ang + teta_ant + ct->argu_d[2] + ct->argu_d[3] - ct->argu_d[4];
      }
      else {
	value = - 180.0 + ang + teta_ant + ct->argu_d[2] + ct->argu_d[3] - ct->argu_d[4];
      }
    }
    if(value>360.0) 
      value -= 360;
    else if(value<-360.0)
	   value += 360;

    p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], value);

    /* cette contrainte n'est pas preparee pour avoir des contraites enchainees */

    return(TRUE);
/*   } */
/*   else { */
/*     PrintInfo(("ERROR: p3d_lowest_z_const : robot can not use this constraint\n"));  */
/*     return(FALSE); */
/*   } */
}

static int p3d_fct_car_front_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  double theta0,thetaR,thetaL,d;
  double min,max;
  int i, I_can;
  double lastvalR,lastvalL;
  static configPt q=NULL;

  if (q==NULL){
    q = p3d_alloc_body_config();
  }

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {
   if(ct->actjnts[0]->num == 0) {
     p3d_get_robot_pos_deg(q);
     theta0 = q[3];
   }
   else
     theta0 = p3d_jnt_get_dof_deg(ct->actjnts[0], ct->act_jnt_dof[0]);
   if(theta0 == 0.0) {
     thetaR = 0.0;
     thetaL = 0.0;
   }
   else {
     d = ct->argu_d[0]/fabs(tan((M_PI/180.0)*theta0));
     if(theta0 < 0.0) {
       thetaR = (180.0/M_PI)*atan2(ct->argu_d[0],d+(ct->argu_d[1])/2);
       thetaL = (180.0/M_PI)*atan2(ct->argu_d[0],d-(ct->argu_d[1])/2);
     }
     else {
       thetaR = -(180.0/M_PI)*atan2(ct->argu_d[0],d-(ct->argu_d[1])/2);
       thetaL = -(180.0/M_PI)*atan2(ct->argu_d[0],d+(ct->argu_d[1])/2);
     }	
   }
   
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((thetaR > max)||(thetaR < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(ct->pasjnts[1],ct->pas_jnt_dof[1],&min,&max);
   if((thetaL > max)||(thetaL < min)) return(FALSE);

   lastvalR = p3d_jnt_get_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   lastvalL = p3d_jnt_get_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1]);
   p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], thetaR);
   p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], thetaL);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	 // multiple iksol is not compatible (yet) with enchained !
	 I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalR);
       p3d_jnt_set_dof_deg(ct->pasjnts[1],ct->pas_jnt_dof[1], lastvalL);
       return(FALSE);
     }
   }
 }
 return(TRUE);
}

static int p3d_fct_cycab_wheels(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  double d, phi, phi1, phi2, phi3, phi4;
  double min,max;
  int i, I_can;
  double lastvalPhi1, lastvalPhi2, lastvalPhi3, lastvalPhi4;
  double l1 = ct->argu_d[0];
  double l2 = ct->argu_d[1];
  double e = ct->argu_d[2];
  static configPt q=NULL;

  if (q==NULL){
    q = p3d_alloc_body_config();
  }

/*  if((j->num == 0)||(j->num == ct->actjnts[0])||(j->num == ct->pasjnts[0])) { */
 if(p3d_go_into_cntrt_fct(ct)) {
   if(ct->actjnts[0]->num == 0) {
     p3d_get_robot_pos(q);
     phi = q[3];
   }
   else
     phi = p3d_jnt_get_dof(ct->actjnts[0], ct->act_jnt_dof[0]);
   if(phi == 0.0) {
     phi1 = 0.0;
     phi2 = 0.0;
     phi3 = 0.0;
     phi4 = 0.0;
   }
   else {
     d = l1/tan(phi);

     phi1 = atan(l1/(d-e/2));
     phi2 = atan(l1/(d+e/2));
     phi3 = -atan(l2/(d-e/2));
     phi4 = -atan(l2/(d+e/2));
   }
   
   p3d_jnt_get_dof_bounds(ct->pasjnts[0],ct->pas_jnt_dof[0],&min,&max);
   if((phi1 > max)||(phi1 < min)) return(FALSE);

   p3d_jnt_get_dof_bounds(ct->pasjnts[1],ct->pas_jnt_dof[1],&min,&max);
   if((phi2 > max)||(phi2 < min)) return(FALSE);

   p3d_jnt_get_dof_bounds(ct->pasjnts[2],ct->pas_jnt_dof[2],&min,&max);
   if((phi3 > max)||(phi4 < min)) return(FALSE);

   p3d_jnt_get_dof_bounds(ct->pasjnts[3],ct->pas_jnt_dof[3],&min,&max);
   if((phi4 > max)||(phi4 < min)) return(FALSE);

   lastvalPhi1 = p3d_jnt_get_dof(ct->pasjnts[0],ct->pas_jnt_dof[0]);
   lastvalPhi2 = p3d_jnt_get_dof(ct->pasjnts[1],ct->pas_jnt_dof[1]);
   lastvalPhi3 = p3d_jnt_get_dof(ct->pasjnts[2],ct->pas_jnt_dof[2]);
   lastvalPhi4 = p3d_jnt_get_dof(ct->pasjnts[3],ct->pas_jnt_dof[3]);

   p3d_jnt_set_dof(ct->pasjnts[0],ct->pas_jnt_dof[0], phi1);
   p3d_jnt_set_dof(ct->pasjnts[1],ct->pas_jnt_dof[1], phi2);
   p3d_jnt_set_dof(ct->pasjnts[2],ct->pas_jnt_dof[2], phi3);
   p3d_jnt_set_dof(ct->pasjnts[3],ct->pas_jnt_dof[3], phi4);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	  // multiple iksol is not compatible (yet) with enchained !
	  I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof(ct->pasjnts[0],ct->pas_jnt_dof[0], lastvalPhi1);
       p3d_jnt_set_dof(ct->pasjnts[1],ct->pas_jnt_dof[1], lastvalPhi2);
       p3d_jnt_set_dof(ct->pasjnts[2],ct->pas_jnt_dof[2], lastvalPhi3);
       p3d_jnt_set_dof(ct->pasjnts[3],ct->pas_jnt_dof[3], lastvalPhi4);
       return(FALSE);
     }
   }
 }
 return(TRUE);
}


/* ------------------------------------------------------------------ */

static int p3d_fct_planar_closed_chain(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 p3d_jnt *JP1, *JP2, *JE2, *Jref;
 double min,max;
 double xP1,yP1,zP1,xE2,yE2,zE2,xref,yref,zref;
 double distance,angACy,theta1,theta2,theta3,chi;
 double lasttheta1,lasttheta2,lasttheta3;
 p3d_vector3 posi_jnt;
 p3d_vector3 v1,v2,v1vectv2,sense;
 int i, I_can;

 JP1 = ct->pasjnts[2];
 JP2 = ct->pasjnts[3];
 JE2 = ct->pasjnts[1];
 
 /*  if(j->num != 0) { */
 p3d_update_this_robot_pos_without_cntrt_and_obj(JP1->rob);
/*  } */
 
 if((JP1->prev_jnt->type == P3D_TRANSLATE) && 
    ((JP1->prev_jnt->p0.x == JP1->p0.x)&&(JP1->prev_jnt->p0.y == JP1->p0.y)&&(JP1->prev_jnt->p0.z == JP1->p0.z)))
   Jref = JP1->prev_jnt->prev_jnt;
 else
   Jref = JP1->prev_jnt;
 
 p3d_jnt_get_cur_vect_point(JE2,posi_jnt); 
 xE2 = posi_jnt[0];
 yE2 = posi_jnt[1];
 zE2 = posi_jnt[2];
 p3d_jnt_get_cur_vect_point(JP1,posi_jnt); 
 xP1 = posi_jnt[0];
 yP1 = posi_jnt[1];
 zP1 = posi_jnt[2];
 p3d_jnt_get_cur_vect_point(Jref,posi_jnt); 
 xref = posi_jnt[0];
 yref = posi_jnt[1];
 zref = posi_jnt[2];
 distance = sqrt(sqr(xP1-xE2)+sqr(yP1-yE2)+sqr(zP1-zE2));

/*  PrintInfo(("%f\n",distance)); */
 if((distance-0.01 > (ct->argu_d[0]+ct->argu_d[1]))||(distance < fabs(ct->argu_d[0]-ct->argu_d[1]))) {
/*  PrintInfo(("%f\n",distance)); */
/*    PrintInfo(("FALSE\n"));   */
   return(FALSE);
 }
 else {
   v1[0] = xref-xP1; 
   v1[1] = yref-yP1;
   v1[2] = zref-zP1;  
   v2[0] = xE2-xP1; 
   v2[1] = yE2-yP1;
   v2[2] = zE2-zP1;
   angACy = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
   p3d_vectXprod(v1,v2,v1vectv2);
   p3d_vectNormalize(v1vectv2,sense);

   p3d_jnt_get_dof_cur_axis(JP1, 0, posi_jnt);
   p3d_vectSub(posi_jnt,sense,v1vectv2); 
   if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) angACy = -angACy;    
   
   theta1 = (180.0/M_PI)*acos((sqr(distance)+sqr(ct->argu_d[0])-sqr(ct->argu_d[1]))/(2*distance*ct->argu_d[0]));
   theta2 = (180.0/M_PI)*asin(ct->argu_d[0]*sin((M_PI/180.0)*theta1)/ct->argu_d[1]);

   p3d_jnt_get_cur_vect_point(JE2->prev_jnt,posi_jnt); 
   xref = posi_jnt[0];
   yref = posi_jnt[1];
   zref = posi_jnt[2];
   v2[0] = -v2[0]; 
   v2[1] = -v2[1];
   v2[2] = -v2[2];  
   v1[0] = xE2-xref; 
   v1[1] = yE2-yref;
   v1[2] = zE2-zref;   
   theta3 = (180.0/M_PI)*acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
   p3d_vectXprod(v1,v2,v1vectv2);
   p3d_vectNormalize(v1vectv2,sense);

   p3d_jnt_get_dof_cur_axis(JE2, 0, posi_jnt);
   p3d_vectSub(posi_jnt,sense,v1vectv2); 
   if((fabs(v1vectv2[0])>0.0001)||(fabs(v1vectv2[1])>0.0001)||(fabs(v1vectv2[2])>0.0001)) theta3 = -theta3;

   if((ct->argu_d[0]*cos((M_PI/180.0)*theta1))>distance)
     theta2 = 180.0 - theta2;
   
   chi = 180.0 - (theta1 + theta2);
   
   if(ct->argu_i[0] == 0) {
     theta3 = theta3 + theta2;
     theta1 = -theta1 + angACy - ct->argu_d[3];
     theta2 = -chi - ct->argu_d[4];
   }
   else {
     theta3 = theta3 - theta2;
     theta1 = theta1 + angACy - ct->argu_d[3];
     theta2 = chi - ct->argu_d[4];
   }

   p3d_jnt_get_dof_bounds_deg(JP1, 0, &min, &max);
   if((min>=-180)&&(max<=180)) {
     if(theta1 < -180)
       theta1 += 360;
     else if(theta1 > 180)
       theta1 -= 360;
   }
   if((theta1 > max)||(theta1 < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(JP2, 0, &min, &max);
   if((min>=-180)&&(max<=180)) {
     if(theta2 < -180)
       theta2 += 360;
     else if(theta2 > 180)
       theta2 -= 360;
   }
   if((theta2 > max)||(theta2 < min)) return(FALSE);
   p3d_jnt_get_dof_bounds_deg(JE2, 0, &min, &max);
   if((min>=-180)&&(max<=180)) {
     if(theta3 < -180)
       theta3 += 360;
     else if(theta3 > 180)
       theta3 -= 360;
   }
   if((theta3 > max)||(theta3 < min)) return(FALSE);   
     
   lasttheta1 = p3d_jnt_get_dof_deg(JP1,0);
   lasttheta2 = p3d_jnt_get_dof_deg(JP2,0);
   lasttheta3 = p3d_jnt_get_dof_deg(JE2,0);
   p3d_jnt_set_dof_deg(JP1,0, theta1);
   p3d_jnt_set_dof_deg(JP2,0, theta2);
   p3d_jnt_set_dof_deg(JE2,0, theta3);

   if(ct->enchained != NULL) {
     I_can = 1;
     for(i=0; I_can && (i<ct->nenchained); i++) {
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],1);
       if(ct->enchained[i]->active)
	  // multiple iksol is not compatible (yet) with enchained !
	  I_can = (*ct->enchained[i]->fct_cntrt)(ct->enchained[i],-1,qp,dl);
       p3d_change_act_rob_dof_state(ct->enchained[i],ct->enchained_rob_dof[i],0);
     }
     if(!I_can) {
       p3d_jnt_set_dof_deg(JP1,0, lasttheta1);
       p3d_jnt_set_dof_deg(JP2,0, lasttheta2);
       p3d_jnt_set_dof_deg(JE2,0, lasttheta3);
       return(FALSE);
     }
   } 
 }
 return(TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for IK (of 3R arm) -- */
static int p3d_set_3R_arm_ik(p3d_cntrt_management * cntrt_manager,
			     int nb_pas, p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double *Dval, int *Ival, int ct_num, int state)
{ 
  p3d_cntrt *ct;
  p3d_matrix4 invT;


  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_3R_ARM_NAME, 
				   3, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				   1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }

    ct->fct_cntrt = p3d_fct_3R_arm_ik;
    ct->ndval = 3; /*???????????????????*/
    ct->nival = 1; /*???????????????????*/

    ct->col_pairs[0][0] = ct->actjnts[0]->o; 
    ct->col_pairs[1][0] = ct->pasjnts[2]->o; 

  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = Dval[0];   /* r1 */
  ct->argu_d[1] = Dval[1];   /* r2 */
  ct->argu_d[2] = Dval[2];   /* gtoq3 */

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

  ct->argu_i[0] = Ival[0];   /* up/down */

  /* transformation between the preceding joint and the base jnt */
  p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->pos0,invT); 
  p3d_mat4Mult(invT,ct->pasjnts[0]->abs_pos,ct->Tbase);    

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);

}



static int p3d_fct_3R_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{ 
  p3d_rob *r;
  double min,max; 
  int i,j;
  p3d_matrix4 Tbase,Tgrip,inv_jnt_mat;
  p3d_matrix4 gripTJ3,TJ3;
  double distance;
  double q[3],qlast[3];
  p3d_vector3 posJ1,posJ3;
  p3d_vector3 pos_diff,J1J3,vprod;
  p3d_vector3 y_axis = {0,1,0};
  p3d_vector3 z_axis = {0,0,1};
  p3d_vector3 l2_axis,l3_axis;
  double distJ1J3;
  double angref,theta1,theta2,theta3;

  r = ct->pasjnts[0]->rob;
  
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  
  p3d_mat4Copy(ct->actjnts[0]->abs_pos,Tbase);
  p3d_mat4Mult(Tbase,ct->Tatt,Tgrip);
  p3d_matInvertXform(ct->pasjnts[0]->jnt_mat, inv_jnt_mat); 
  p3d_mat4Mult(ct->pasjnts[0]->abs_pos, inv_jnt_mat, Tbase);

  distance = sqrt(sqr(Tgrip[0][3]-Tbase[0][3])+sqr(Tgrip[1][3]-Tbase[1][3])+sqr(Tgrip[2][3]-Tbase[2][3]));
/*   printf(("distance = %f\n",distance); */

  // check if end-frame is in workspace
  if((distance > ct->argu_d[MAX_ARGU_CNTRT - 1]) || 
     (distance < ct->argu_d[MAX_ARGU_CNTRT - 2])) {
    return(FALSE);
    //return(TRUE);
  }

  // WARNING : suppose that robot in home position is aligned with y-axis (-> REFERENCES)
  p3d_mat4Copy(p3d_mat4IDENTITY, gripTJ3);
  gripTJ3[1][3] = - ct->argu_d[2];
  p3d_mat4Mult(Tgrip,gripTJ3,TJ3);

  posJ1[0] = Tbase[0][3];
  posJ1[1] = Tbase[1][3];
  posJ1[2] = Tbase[2][3];
  posJ3[0] = TJ3[0][3];
  posJ3[1] = TJ3[1][3];
  posJ3[2] = TJ3[2][3];

  p3d_vectSub(posJ3,posJ1,pos_diff);  
  distJ1J3 = p3d_vectNorm(pos_diff);
  p3d_vectNormalize(pos_diff,J1J3);
  angref = acos(p3d_vectDotProd(y_axis,J1J3));
  p3d_vectXprod(y_axis,J1J3,vprod);
  if(!p3d_same_sign_vect(vprod,z_axis)) 
    angref = -angref;

  theta1 = acos((sqr(distJ1J3)+sqr(ct->argu_d[0])-sqr(ct->argu_d[1]))/(2*distJ1J3*ct->argu_d[0]));
  theta2 = acos((sqr(ct->argu_d[0])+sqr(ct->argu_d[1])-sqr(distJ1J3))/(2*ct->argu_d[0]*ct->argu_d[1])) - M_PI;

  if(ct->argu_i[0] == -1) {
    theta1 = -theta1;
    theta2 = -theta2;
  }
  theta1 += angref;

  l2_axis[0] = -sin(theta1 + theta2);
  l2_axis[1] = cos(theta1 + theta2);
  l2_axis[2] = 0;

  l3_axis[0] = Tgrip[0][1];
  l3_axis[1] = Tgrip[1][1];
  l3_axis[2] = Tgrip[2][1];

  theta3 = acos(p3d_vectDotProd(l2_axis,l3_axis)/(p3d_vectNorm(l2_axis)*p3d_vectNorm(l3_axis)));
  p3d_vectXprod(l2_axis,l3_axis,vprod);
  if(!p3d_same_sign_vect(vprod,z_axis)) 
    theta3 = -theta3;

  //printf("theta1 = %f\n",theta1);
  //printf("theta2 = %f\n",theta2);
  //printf("theta3 = %f\n",theta3);

  q[0] = theta1;
  q[1] = theta2;
  q[2] = theta3;

  // check joint ranges    
  for(i=0; i<ct->npasjnts; i++) {
    p3d_jnt_get_dof_bounds(ct->pasjnts[i],ct->pas_jnt_dof[i], &min, &max);
    qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
    if((q[i] <= max)&&(q[i] >= min)) {
      p3d_jnt_set_dof(ct->pasjnts[i],ct->pas_jnt_dof[i], q[i]);
    }
    else {
      for(j=0; j<i; j++) {
	p3d_jnt_set_dof(ct->pasjnts[j],ct->pas_jnt_dof[j], qlast[j]);
      }
      return(FALSE);
      //return(TRUE);
    } 
  }
  
  return(TRUE);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for IK (of R6arm) -- */
static int p3d_set_R6_arm_ik(p3d_cntrt_management * cntrt_manager,
			     int nb_pas, p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double *Dval, int *Ival, int ct_num, int state)
{ 
  p3d_cntrt *ct;
  p3d_jnt * jnt_arrPt[6];
  int jnt_dof_arrPt[6];
  int rob_dof_arrPt[6];
  p3d_jnt * jnt_prevPt, * jntPt;
  int i;
  p3d_matrix4 invT;


  if (nb_pas == 6) {
    for(i=0; i<6; i++) {
      if (pas_jntPt[i]->type != P3D_ROTATE)
	{ return FALSE; }
      jnt_arrPt[i] = pas_jntPt[i];
      jnt_dof_arrPt[i] = pas_jnt_dof[i];
      rob_dof_arrPt[i] = pas_rob_dof[i];
    }
  } else {
    jntPt = jnt_prevPt = pas_jntPt[0];
    for(i=0; i<6; i++) {
      if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || 
	  (jntPt->type != P3D_ROTATE) || ((i<5) && (jntPt->n_next_jnt!=1)))
	{ return FALSE; }
      jnt_arrPt[i] = jntPt;
      jnt_dof_arrPt[i] = 0;
      rob_dof_arrPt[i] = pas_rob_dof[0] +
	jntPt->index_dof - jnt_arrPt[0]->index_dof;
      
      jnt_prevPt = jntPt;
      if(jntPt->next_jnt == NULL)
	jntPt = NULL;
      else
	jntPt = jntPt->next_jnt[0];
    }
  }

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_R6_ARM_NAME, 
				     6, jnt_arrPt, jnt_dof_arrPt, rob_dof_arrPt,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }

    ct->fct_cntrt = p3d_fct_R6_arm_ik;
    ct->ndval = 3; /*???????????????????*/
    ct->nival = 3; /*???????????????????*/

    ct->col_pairs[0][0] = ct->actjnts[0]->o; 
    ct->col_pairs[1][0] = ct->pasjnts[5]->o; 

  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }

  ct->argu_d[0] = Dval[0];   /* a2 */
  ct->argu_d[1] = Dval[1];   /* r4 */
  ct->argu_d[2] = Dval[2];   /* gtoq6 */

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

  ct->argu_i[0] = Ival[0];   /* e1 */
  ct->argu_i[1] = Ival[1];   /* e3 */
  ct->argu_i[2] = Ival[2];   /* e5 */

  /* transformation between the preceding joint and the base jnt */
  p3d_matInvertXform(ct->pasjnts[0]->prev_jnt->pos0,invT); 
  p3d_mat4Mult(invT,ct->pasjnts[0]->abs_pos,ct->Tbase);    

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);

}



static int p3d_fct_R6_arm_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{ 
  p3d_rob *r;
  double min,max; 
  int i,j;
  p3d_matrix4 Tbase,Tgrip,inv_jnt_mat;
  double distance;
  double q[6],qlast[6];

  r = ct->pasjnts[0]->rob;

  if(!TEST_PHASE) {
    p3d_update_this_robot_pos_without_cntrt_and_obj(ct->pasjnts[0]->rob);
    /* necesario ???????? */  /* solo si no es en generacion ??????? */ 
  }
  
  p3d_mat4Copy(ct->actjnts[0]->abs_pos,Tbase);
  p3d_mat4Mult(Tbase,ct->Tatt,Tgrip);
  p3d_matInvertXform(ct->pasjnts[0]->jnt_mat, inv_jnt_mat); 
  p3d_mat4Mult(ct->pasjnts[0]->abs_pos, inv_jnt_mat, Tbase);

  
  distance = sqrt(sqr(Tgrip[0][3]-Tbase[0][3])+sqr(Tgrip[1][3]-Tbase[1][3])+sqr(Tgrip[2][3]-Tbase[2][3]));
/*   printf(("distance = %f\n",distance); */

  /* FALTA : */
  /* - controlar limites */
  /* - referencias angulares (generalizar) */

 
/*   if(!compute_inverse_kinematics_R6_arm (q,r->joints[ct->actjnts[0]]->abs_pos,Tbase, */
  if(!compute_inverse_kinematics_R6_arm (q,Tgrip,Tbase,
					 ct->argu_d[0],ct->argu_d[1],ct->argu_d[2],
					 ct->argu_i[0],ct->argu_i[1],ct->argu_i[2])) {
    return(FALSE);
  }
  
  else {
    
/*     p3d_set_robot_jnt(ct->pasjnts[0], q[0]); */
/*     p3d_set_robot_jnt(ct->pasjnts[1], q[1]-M_PI/2); */
/*     p3d_set_robot_jnt(ct->pasjnts[2], q[2]-M_PI/2); */
/*     p3d_set_robot_jnt(ct->pasjnts[3], q[3]); */
/*     p3d_set_robot_jnt(ct->pasjnts[4], q[4]); */
/*     p3d_set_robot_jnt(ct->pasjnts[5], q[5]); */

    for(i=0; i<ct->npasjnts; i++) {
      p3d_jnt_get_dof_bounds(ct->pasjnts[i],ct->pas_jnt_dof[i], &min, &max);
      qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]);
      if((q[i] <= max)&&(q[i] >= min)) {
	p3d_jnt_set_dof(ct->pasjnts[i],ct->pas_jnt_dof[i], q[i]);
      }
      else {
	for(j=0; j<i; j++) {
	  p3d_jnt_set_dof(ct->pasjnts[j],ct->pas_jnt_dof[j], qlast[j]);
	}
	return(FALSE);
     } 
    }
  }
  
  return(TRUE);
}

/****************************************************************************/
/* Functions for prismatic actuator invers kinematic computation */


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
static int p3d_set_prismatic_actuator_ik(p3d_cntrt_management * cntrt_managerPt,
					 p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
					 p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
					 double *Dval, int ct_num, int state)
{ 
  p3d_cntrt *ctPt;
  p3d_matrix4 Tend,invTbase,Tbe;
  double theta_x,theta_y,dist_be;
  double vmin,vmax;

  /* Test the validity of passif joints */
  if ((pas_jntPt[0]->type != pas_jntPt[1]->type) || (pas_jntPt[0]->type != P3D_KNEE)
      || (pas_jntPt[2]->type != P3D_TRANSLATE)) {
    PrintWarning(("Wrong type of passif joints !!!\n"));
    return FALSE;
  }

  if (ct_num < 0) {
    ctPt = p3d_create_generic_cntrts(cntrt_managerPt, CNTRT_PRISMATIC_ACTUATOR_NAME, 
				       3, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				       1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ctPt == NULL)
      { return FALSE; }

    ctPt->fct_cntrt = p3d_fct_prismatic_actuator_ik;

    /* initialize the parameters number */
    ctPt->ndval = 3;
    ctPt->nival = 0;

    ctPt->col_pairs[0][0] = ctPt->actjnts[0]->o; 
    ctPt->col_pairs[1][0] = ctPt->pasjnts[2]->o;

    /* PROVISIONAL !!!!!!!!!!!!! */
    ctPt->Tatt[0][0] = 1.0; ctPt->Tatt[0][1] = 0.0; ctPt->Tatt[0][2] = 0.0; ctPt->Tatt[0][3] = Dval[0]; 
    ctPt->Tatt[1][0] = 0.0; ctPt->Tatt[1][1] = 1.0; ctPt->Tatt[1][2] = 0.0; ctPt->Tatt[1][3] = Dval[1]; 
    ctPt->Tatt[2][0] = 0.0; ctPt->Tatt[2][1] = 0.0; ctPt->Tatt[2][2] = 1.0; ctPt->Tatt[2][3] = Dval[2]; 
    ctPt->Tatt[3][0] = 0.0; ctPt->Tatt[3][1] = 0.0; ctPt->Tatt[3][2] = 0.0; ctPt->Tatt[3][3] = 1.0; 
    /* !!!!!!!!!!!!!!!!!!!!!! */

    p3d_mat4Mult(ctPt->actjnts[0]->abs_pos,ctPt->Tatt,Tend);
    p3d_matInvertXform(ctPt->pasjnts[1]->pos0,invTbase); 
    p3d_mat4Mult(invTbase,Tend,Tbe);
    
    theta_x = - atan2(Tbe[1][3], Tbe[2][3]);
    theta_y = atan2(Tbe[0][3], (Tbe[2][3]/cos(theta_x)));
    dist_be = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3]));
 
    ctPt->argu_d[0] = theta_x - p3d_jnt_get_dof_deg(ctPt->pasjnts[0], ctPt->pas_jnt_dof[0]);
    ctPt->argu_d[1] = theta_y - p3d_jnt_get_dof_deg(ctPt->pasjnts[1], ctPt->pas_jnt_dof[1]);
    ctPt->argu_d[2] = dist_be - p3d_jnt_get_dof_deg(ctPt->pasjnts[2], ctPt->pas_jnt_dof[2]);

    /* max. extension */
    p3d_jnt_get_dof_bounds(ctPt->pasjnts[2],ctPt->pas_jnt_dof[2],&vmin,&vmax);
    ctPt->argu_d[MAX_ARGU_CNTRT - 1] = ctPt->argu_d[2] + vmax;
    /* min. extension */
    ctPt->argu_d[MAX_ARGU_CNTRT - 2] = ctPt->argu_d[2];

  } else {
    ctPt = cntrt_managerPt->cntrts[ct_num];
  }

  if ((!state) || (!(ctPt->active) && state)) {
    if (!p3d_update_jnts_state(ctPt, state))
      { return FALSE; }
  }
  ctPt->active = state;
  last_cntrt_set = ctPt;
  return(TRUE);
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
static int p3d_fct_prismatic_actuator_ik(p3d_cntrt *ctPt, int iksol, configPt qp, double dl)
{ 
  double min, max; 
  p3d_matrix4 Tend,invTbase,Tbe;
  double q[3], qlast[3];
  int i,j;

  //  if(p3d_go_into_cntrt_fct(ctPt)) {

  if(!TEST_PHASE) {
    p3d_update_this_robot_pos_without_cntrt_and_obj(ctPt->pasjnts[0]->rob);
  }

    p3d_mat4Mult(ctPt->actjnts[0]->abs_pos,ctPt->Tatt,Tend);
    p3d_matInvertXform(ctPt->pasjnts[1]->pos0,invTbase); 
    p3d_mat4Mult(invTbase,Tend,Tbe);
    
    q[0] = - atan2(Tbe[1][3], Tbe[2][3]) - ctPt->argu_d[0]; 
    q[1] = atan2(Tbe[0][3], (Tbe[2][3]/cos(q[0]))) - ctPt->argu_d[1];
    q[2] = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3])) - ctPt->argu_d[2];
 
    for(i=0; i<ctPt->npasjnts; i++) {
      p3d_jnt_get_dof_bounds(ctPt->pasjnts[i],ctPt->pas_jnt_dof[i], &min, &max);
      qlast[i] = p3d_jnt_get_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i]);
      if((q[i] <= max)&&(q[i] >= min)) {
	p3d_jnt_set_dof(ctPt->pasjnts[i],ctPt->pas_jnt_dof[i], q[i]);
      } else {
	for(j=0; j<i; j++) {
	  p3d_jnt_set_dof(ctPt->pasjnts[j],ctPt->pas_jnt_dof[j], qlast[j]);
	}
	return(FALSE);
      } 
    }
    //  }    
  return(TRUE);
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
static int p3d_set_prismatic_actuator_II_ik(p3d_cntrt_management * cntrt_managerPt,
					    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
					    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
					    double l0, int ct_num, int state)
{ 
  p3d_cntrt *ctPt;
  double vmin,vmax;
  p3d_matrix4 invT;

  /* Test the validity of passif joints */
  if ((pas_jntPt[0]->type != pas_jntPt[1]->type) || (pas_jntPt[0]->type != P3D_KNEE)
      || (pas_jntPt[2]->type != P3D_TRANSLATE)) {
    PrintWarning(("Wrong type of passif joints !!!\n"));
    return FALSE;
  }

  if (ct_num < 0) {
    ctPt = p3d_create_generic_cntrts(cntrt_managerPt, CNTRT_PRISMATIC_ACTUATOR_II_NAME, 
				       3, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				       1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ctPt == NULL)
      { return FALSE; }

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
    p3d_jnt_get_dof_bounds(ctPt->pasjnts[2],ctPt->pas_jnt_dof[2],&vmin,&vmax);
    ctPt->argu_d[MAX_ARGU_CNTRT - 1] = l0 + vmax;
    /* min. extension */
    ctPt->argu_d[MAX_ARGU_CNTRT - 2] = l0;

    /* transformation between the preceding joint and the base knee */
    p3d_matInvertXform(ctPt->pasjnts[0]->prev_jnt->pos0,invT); 
    p3d_mat4Mult(invT,ctPt->pasjnts[0]->pos0,ctPt->Tbase);    


  } else {
    ctPt = cntrt_managerPt->cntrts[ct_num];
  }

  if ((!state) || (!(ctPt->active) && state)) {
    if (!p3d_update_jnts_state(ctPt, state))
      { return FALSE; }
  }
  ctPt->active = state;
  last_cntrt_set = ctPt;
  return(TRUE);
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
static int p3d_fct_prismatic_actuator_II_ik(p3d_cntrt *ctPt, int iksol, configPt qp, double dl)
{ 
  double min, max; 
  p3d_matrix4 invTbase,Tb,Tbe;
  double q[3], qlast[3];
  int i,j;

  //  if(p3d_go_into_cntrt_fct(ctPt)) {

  if(!TEST_PHASE) {
    p3d_update_this_robot_pos_without_cntrt_and_obj(ctPt->pasjnts[0]->rob);
  }
  
  p3d_mat4Mult(ctPt->pasjnts[0]->prev_jnt->abs_pos,ctPt->Tbase,Tb);
  p3d_matInvertXform(Tb,invTbase); 
  p3d_mat4Mult(invTbase,ctPt->actjnts[0]->abs_pos,Tbe);
  
  /* WARNING !!! :
     the configuration of the attachment-joint is not computed
  */
  
  q[0] = - atan2(Tbe[1][3], Tbe[2][3]); 
  q[1] = atan2(Tbe[0][3], (Tbe[2][3]/cos(q[0])));
  q[2] = sqrt(SQR(Tbe[0][3]) + SQR(Tbe[1][3]) + SQR(Tbe[2][3])) - ctPt->argu_d[2];
  
  for(i=0; i<ctPt->npasjnts; i++) {
    p3d_jnt_get_dof_bounds(ctPt->pasjnts[i],ctPt->pas_jnt_dof[i], &min, &max);
    qlast[i] = p3d_jnt_get_dof(ctPt->pasjnts[i], ctPt->pas_jnt_dof[i]);
    if((q[i] <= max)&&(q[i] >= min)) {
      p3d_jnt_set_dof(ctPt->pasjnts[i],ctPt->pas_jnt_dof[i], q[i]);
    } else {
      for(j=0; j<i; j++) {
	p3d_jnt_set_dof(ctPt->pasjnts[j],ctPt->pas_jnt_dof[j], qlast[j]);
      }
      return(FALSE);
    } 
  }
  //  }    
  return(TRUE);
}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for CTBot IK  -- */
static int p3d_set_CTBot_ik(p3d_cntrt_management * cntrt_manager,
			    p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			    p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			    double *Dval, int ct_num, int state)
{ 
  p3d_rob *robPt;
  p3d_cntrt *ctPt=NULL;
  //double vmin,vmax;
  //p3d_matrix4 invT;

  // validate joints (TO DO)


  if (ct_num < 0) {
    ctPt = p3d_create_generic_cntrts(cntrt_manager, CNTRT_CTBOT_NAME, 
				     13, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     1, act_jntPt, act_jnt_dof, act_rob_dof);

    if (ctPt == NULL)
      { return FALSE; }
    
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
    p3d_mat4Copy(robPt->joints[0]->pos0,ctPt->Tbase);
    
  } else {
    ctPt = cntrt_manager->cntrts[ct_num];
  }

  if ((!state) || (!(ctPt->active) && state)) {
    if (!p3d_update_jnts_state(ctPt, state))
      { return FALSE; }
  }
  ctPt->active = state;
  last_cntrt_set = ctPt;
  return(TRUE);
}


static int p3d_fct_CTBot_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{ 
  p3d_rob *r;
  //double min,max; 
  int i;
  p3d_matrix4 Tbase,invTbase,Tplatf,Ff;
  //double f[3], z[3];
  double q[13];
  //double qlast[13];

  r = ct->pasjnts[0]->rob;

  //if(p3d_go_into_cntrt_fct(ct)) {
    p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[0]->rob);
  //}

  p3d_mat4Copy(ct->actjnts[0]->abs_pos,Tplatf);
  p3d_mat4Copy(ct->Tbase,Tbase);
  p3d_matInvertXform(Tbase,invTbase); 
  p3d_mat4Mult(invTbase,Tplatf,Ff);
 
/*   f[0] = Tplatf[0][3]-Tbase[0][3]; */
/*   f[1] = Tplatf[1][3]-Tbase[1][3]; */
/*   f[2] = Tplatf[2][3]-Tbase[2][3]; */

/*   z[0] = Tplatf[0][2]; */
/*   z[1] = Tplatf[1][2]; */
/*   z[2] = Tplatf[2][2]; */

  if(!CTBot_MGI(Ff,q)) {
    return(FALSE);
  }
  else {
    for(i=0; i<ct->npasjnts; i++) {
/*       p3d_jnt_get_dof_bounds(ct->pasjnts[i],ct->pas_jnt_dof[i], &min, &max); */
/*       qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], ct->pas_jnt_dof[i]); */
/*       if((q[i] <= max)&&(q[i] >= min)) { */
/* 	p3d_jnt_set_dof(ct->pasjnts[i],ct->pas_jnt_dof[i], q[i]); */
/*       } */
/*       else { */
/* 	for(j=0; j<i; j++) { */
/* 	  p3d_jnt_set_dof(ct->pasjnts[j],ct->pas_jnt_dof[j], qlast[j]); */
/* 	} */
/* 	return(FALSE); */
/*       } */
      //// NO JOINT LIMITS !!! ///////
      p3d_jnt_set_dof(ct->pasjnts[i],ct->pas_jnt_dof[i], q[i]);
      //////////////////////////////// 
    }
  }
  
  return(TRUE);
}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for RLG test -- */
static int p3d_set_in_sphere(p3d_cntrt_management * cntrt_manager,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     double x, double y, double z, int ct_num, int state)
{ 
  p3d_cntrt *ct;

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_IN_SPHERE_NAME, 
				     0, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				     0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_in_sphere;
  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  ct->argu_d[0] = x;
  ct->argu_d[1] = y;
  ct->argu_d[2] = z;

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_in_sphere(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
 p3d_jnt *JE;
 p3d_vector3 posi_jnt;
 p3d_vector3 v1,X1,X2;
 double distance;

 JE = ct->actjnts[0];

 if(!p3d_get_RLG())
   p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);

 p3d_jnt_get_cur_vect_point(JE, posi_jnt);
 X1[0] = posi_jnt[0];
 X1[1] = posi_jnt[1];
 X1[2] = posi_jnt[2];
 X2[0] = ct->argu_d[0];
 X2[1] = ct->argu_d[1];
 X2[2] = ct->argu_d[2];
 p3d_vectSub(X2, X1, v1);	
 distance = sqrt(sqr(v1[0])+sqr(v1[1])+sqr(v1[2])); 
/*  printf("ditance_fct: %f\n",distance); */

 if(distance > ct->argu_d[3]) {
   return(FALSE);
 }
 return TRUE;
}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for cntrt limiting the distance range of 2 jnts relative position -- */

static int p3d_set_jnts_relpos_bound(p3d_cntrt_management * cntrt_manager,
				     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				     double *Dval, int *Ival, int ct_num, int state)
{ 
  p3d_cntrt *ct;
  p3d_rob *robPt;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_JNTS_RELPOS_BOUND, 
				   0, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				   0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_jnts_relpos_bound;

    // indices of the 2 jnts
    ct->argu_i[0] = Ival[0]; // joint of the 1st jnt
    ct->argu_i[1] = Ival[1]; // joint of the 2nd jnt
  } 
  else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  ct->argu_d[0] = Dval[0];
  ct->argu_d[1] = Dval[1];

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_jnts_relpos_bound(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *J1,*J2;
  p3d_vector3 posJ1,posJ2,pos_diff;
  double distance;

  // WARNING : this shold not be made : can give problems for multiple robots 
  p3d_rob *robPt;
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ////////////

  if(!p3d_get_RLG())
    p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ???

  J1 = robPt->joints[ct->argu_i[0]];
  J2  = robPt->joints[ct->argu_i[1]];

  p3d_jnt_get_cur_vect_point(J1,posJ1);
  p3d_jnt_get_cur_vect_point(J2,posJ2);

  p3d_vectSub(posJ2,posJ1,pos_diff);	
  distance = (double) p3d_vectNorm(pos_diff);

  //printf("ctnum : %d  , distance : %f\n",ct->num,distance);
  
  if((distance < ct->argu_d[0]) || (distance > ct->argu_d[1])) {
    return(FALSE);
  }

  return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for 6R_bio_ik -- */
static int p3d_set_6R_bio_ik(p3d_cntrt_management * cntrt_manager,
			     p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
			     p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
			     int ct_num, int state)
{ 
  p3d_jnt *jnt_prevPt, *jntPt;
  p3d_cntrt *ct;
  p3d_jnt * jnt_arrPt[6];
  int jnt_dof_arrPt[6];
  int rob_dof_arrPt[6];
  int i;

  /* set passive joints */
  jntPt = jnt_prevPt = pas_jntPt[0];
  for(i=0; i<6; i++) {
    if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || 
	(jntPt->type != P3D_ROTATE))
      { return FALSE; }
    jnt_arrPt[i] = jntPt;
    jnt_dof_arrPt[i] = 0;
    rob_dof_arrPt[i] = pas_rob_dof[0] +
      jntPt->index_dof - jnt_arrPt[0]->index_dof;
    
    jnt_prevPt = jntPt;
    if(jntPt->next_jnt == NULL)
      jntPt = NULL;
    else
      // WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
      jntPt = jntPt->next_jnt[0];
  } 
   
  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_6R_BIO_IK_NAME, 
				   6, jnt_arrPt, jnt_dof_arrPt, rob_dof_arrPt,
				   1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }

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
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_6R_bio_ik(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *JE,*J;
  double **sol_configs;
  double **valid_sol_configs;
  int i;
  double min,max;
  double qlast[6];
  int nsol,nvalidsol,ns,imds;
  int fail;
  double pqdist,minpqdist,aqdist;
  configPt qc,qcm;
  double dlfact;
  double ljnt;
  int naj;


  // local iksol allocation
  if(st_iksol == NULL)
    alloc_and_init_st_iksol(ct->cntrt_manager);
  
  JE = ct->actjnts[0];
  
  p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);

  sol_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }
  
  /* compute all ik solutions */
  nsol = bio_compute_ik(ct,sol_configs);
  if(!nsol) {
    for(i = 0; i < 16; i++) {
      free(sol_configs[i]);
    }
    free(sol_configs);
    return(FALSE);
/*     return(TRUE); */
  }

  for(i=0; i<6; i++) {
    qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
  }  

  valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
  for(i = 0; i < nsol; i++) {
    valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  

  ns = 0;
  nvalidsol = 0;
  while(ns<nsol) {
    /*     printf("\n"); */
    fail = 0;
    for(i=0; (i<6) && (!fail); i++) {
      p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &min, &max);
      if((sol_configs[ns][i] <= max)&&(sol_configs[ns][i] >= min)) {
	valid_sol_configs[nvalidsol][i] = sol_configs[ns][i];
	/* 	  printf(" %f ",sol_configs[ns][i]); */
	if(i == 5)
	  nvalidsol++;
      }
      else {
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
  if(look_iksol)
    iksol = st_iksol[ct->num];

  if(nvalidsol > 0) {
    if(qp == NULL) {
      if(iksol != -1) {
	/* if specified iksol */
	ns = iksol - 1;
      }
      else {
	/* choice a solution at random */
	ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6));
      }
      for(i=0; i<6 ; i++) {
	p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]);
      }       

      // f modif for alkanes
      nvalidsol = ns + 1;
    }
    else {
      /* choice solution depending on qp */
      /* NOTE : the test of distance is made with the whole conf.
	        this could be a problem when the system has high n dofs
		-> the test should be made only for the passive dofs !!!
      */
      minpqdist = P3D_HUGE;
      qc = p3d_alloc_config(JE->rob);
      qcm = p3d_alloc_config(JE->rob);
      imds = 0;
      for(ns=0; ns<nvalidsol; ns++) {
	for(i=0; i<6; i++) {
	  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]);
	}       	
	p3d_get_robot_config_into(JE->rob, &qc);
	if(DEBUG_CNTRTS) {
	  // visualizar qp y q
	  p3d_set_robot_config(JE->rob,qp);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	  p3d_set_robot_config(JE->rob,qc);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	}
	ljnt = 0.0;
	for(i=0; i<6; i++) {
	  ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc, qp));
	  //printf("ljnt_iter = %f\n",ljnt);
	}
	pqdist = sqrt(ljnt);
	//printf("pqdist_i = %f\n",pqdist);
/* 	qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
	if(pqdist < minpqdist) {
	  minpqdist = pqdist;
	  imds = ns;
	  p3d_copy_config_into(JE->rob,qc,&qcm); 
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
      if((ct->rlgPt != NULL) &&  (ct->rlgPt->rlgchPt != NULL))
	J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
      else
	J = JE->rob->joints[1];
      ljnt = 0.0;
      naj = 0;
      while(J != ct->pasjnts[0]) {
	naj++;
	ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
	//printf("ljnt_iter = %f\n",ljnt);
	// WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
	J = J->next_jnt[0];
      }
      aqdist = sqrt(ljnt);
      //printf("aqdist = %f\n",aqdist);
      dlfact = 500.0;   // ???!!!
      p3d_destroy_config(JE->rob,qc); 
      p3d_destroy_config(JE->rob,qcm); 

      // **************************************
      //IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
      //           y utilizarlo en este test
      // **************************************
      // NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
      //printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
      //if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ???? 
      if((minpqdist) < (dlfact * aqdist)) {
	//printf("  -> OK\n");
	for(i=0; i<6; i++) {
	  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i]);
	}
	nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
      }
      else {
	//printf("  -> FAIL\n");
	nvalidsol = 0;
      }
    }
  }
   
  if(nvalidsol == 0) {
    for(i=0; i<6; i++) {
      p3d_jnt_set_dof(ct->pasjnts[i],0, qlast[i]);
    }
  }
    
  for(i = 0; i < nsol; i++) {
    free(valid_sol_configs[i]);
  }
  free(valid_sol_configs);
  
  for(i = 0; i < 16; i++) {
    free(sol_configs[i]);
  }
  free(sol_configs);

  // local iksol
  st_iksol[ct->num] = nvalidsol; 
  
  return (nvalidsol);
/*   return (TRUE); */
}


static int p3d_fct_6R_bio_ik_alkanes(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *JE,*J;
  double **sol_configs;
  double **valid_sol_configs;
  int i;
  double min,max;
  double qlast[6];
  int nsol,nvalidsol,ns,imds;
  int fail;
  double pqdist,minpqdist,aqdist;
  configPt qc,qcm;
  double dlfact;
  double ljnt;
  int naj;
  /////////////////////
  int j;
  double d1,d2,d3;
  p3d_vector3 pos1,pos2,pos_diff;
  ////////////////////
  //static double mq1=1000,Mq1=-1000,mq2=1000,Mq2=-1000;


  // local iksol allocation
  if(st_iksol == NULL)
    alloc_and_init_st_iksol(ct->cntrt_manager);
  
  JE = ct->actjnts[0];
  
  p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);

  sol_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }
  
  /* compute all ik solutions */
  nsol = bio_compute_ik(ct,sol_configs);
  if(!nsol) {
    for(i = 0; i < 16; i++) {
      free(sol_configs[i]);
    }
    free(sol_configs);
    return(FALSE);
/*     return(TRUE); */
  }

  for(i=0; i<6; i++) {
    qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
  }  

  valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
  for(i = 0; i < nsol; i++) {
    valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  

  ns = 0;
  nvalidsol = 0;
  while(ns<nsol) {
    /*     printf("\n"); */
    fail = 0;
    for(i=0; (i<6) && (!fail); i++) {
      p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &min, &max);
      if((sol_configs[ns][i] <= max)&&(sol_configs[ns][i] >= min)) {
	valid_sol_configs[nvalidsol][i] = sol_configs[ns][i];
	/* 	  printf(" %f ",sol_configs[ns][i]); */
	if(i == 5)
	  nvalidsol++;
      }
      else {
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
  if(look_iksol)
    iksol = st_iksol[ct->num];

  if(nvalidsol > 0) {
    if(qp == NULL) {
      if(iksol != -1) {
	/* if specified iksol */
	ns = iksol - 1;
      }
      // modif for alkanes
      else {
      // modif for alkanes
	////////////////////////
	/* choice a solution at random */
/* 	ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6)); */
/*       } */
/*       for(i=0; i<6 ; i++) { */
/* 	p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]); */
/*       }      */  
	////////////////////////
	////////////////////////
	for(j=0; j < nvalidsol;j++) {
	  for(i=0; i<6 ; i++) {
	    p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[j][i]);
	  }
	  p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);

	  p3d_jnt_get_cur_vect_point(JE->rob->joints[2],pos1);
	  p3d_jnt_get_cur_vect_point(JE->rob->joints[6],pos2);
	  p3d_vectSub(pos1,pos2,pos_diff);
	  d1 = SQR((double) p3d_vectNorm(pos_diff));
	  p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1);
	  p3d_jnt_get_cur_vect_point(JE->rob->joints[6],pos2);
	  p3d_vectSub(pos1,pos2,pos_diff);
	  d2 = SQR((double) p3d_vectNorm(pos_diff));
	  p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1);
	  p3d_jnt_get_cur_vect_point(JE->rob->joints[7],pos2);
	  p3d_vectSub(pos1,pos2,pos_diff);
	  d3 = SQR((double) p3d_vectNorm(pos_diff));


/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[1],pos1); */
/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos2); */
/* 	  p3d_vectSub(pos1,pos2,pos_diff); */
/* 	  d1 = SQR((double) p3d_vectNorm(pos_diff)); */
/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[2],pos1); */
/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[4],pos2); */
/* 	  p3d_vectSub(pos1,pos2,pos_diff); */
/* 	  d2 = SQR((double) p3d_vectNorm(pos_diff)); */
/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[3],pos1); */
/* 	  p3d_jnt_get_cur_vect_point(JE->rob->joints[5],pos2); */
/* 	  p3d_vectSub(pos1,pos2,pos_diff); */
/* 	  d3 = SQR((double) p3d_vectNorm(pos_diff)); */
	  
	  printf("{ VECT 1 1 1 1 1 %f %f %f 255 0 0 1 }\n",d1,d2,d3);

	  // PLOT DE q1 y q2
	  
	  //printf("{ VECT 1 1 1 1 1 %f %f %f 255 0 0 1 }\n",JE->rob->joints[1]->v,1.0,0.0);
	  //printf("{ VECT 1 1 1 1 1 %f %f %f 0 255 0 1 }\n",JE->rob->joints[2]->v,2.0,0.0);

/* 	  mq1 = 1.29; */
/* 	  if(fabs(JE->rob->joints[1]->v) < mq1) { */
/* 	    mq1 = fabs(JE->rob->joints[1]->v); */
/* 	    printf("minq1 = %f\n",mq1); */
/* 	  } */

/* 	  if(JE->rob->joints[1]->v > Mq1) { */
/* 	    Mq1 = JE->rob->joints[1]->v; */
/* 	    printf("maxq1 = %f\n",Mq1); */
/* 	  } */
/* 	  else if(JE->rob->joints[1]->v < mq1) { */
/* 	    mq1 = JE->rob->joints[1]->v; */
/* 	    printf("minq1 = %f\n",mq1); */
/* 	  } */
/* 	  if(JE->rob->joints[2]->v > Mq2) { */
/* 	    Mq2 = JE->rob->joints[2]->v; */
/* 	    printf("maxq2 = %f\n",Mq2); */
/* 	  } */
/* 	  else if(JE->rob->joints[2]->v < mq2) { */
/* 	    mq2 = JE->rob->joints[2]->v; */
/* 	    printf("minq2 = %f\n",mq2); */
/* 	  } */

	}
      }
	////////////////////////

      // f modif for alkanes
      nvalidsol = ns + 1;
    }
    else {
      /* choice solution depending on qp */
      /* NOTE : the test of distance is made with the whole conf.
	        this could be a problem when the system has high n dofs
		-> the test should be made only for the passive dofs !!!
      */
      minpqdist = P3D_HUGE;
      qc = p3d_alloc_config(JE->rob);
      qcm = p3d_alloc_config(JE->rob);
      imds = 0;
      for(ns=0; ns<nvalidsol; ns++) {
	for(i=0; i<6; i++) {
	  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i]);
	}       	
	p3d_get_robot_config_into(JE->rob, &qc);
	if(DEBUG_CNTRTS) {
	  // visualizar qp y q
	  p3d_set_robot_config(JE->rob,qp);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	  p3d_set_robot_config(JE->rob,qc);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	}
	ljnt = 0.0;
	for(i=0; i<6; i++) {
	  ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc, qp));
	  //printf("ljnt_iter = %f\n",ljnt);
	}
	pqdist = sqrt(ljnt);
	//printf("pqdist_i = %f\n",pqdist);
/* 	qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
	if(pqdist < minpqdist) {
	  minpqdist = pqdist;
	  imds = ns;
	  p3d_copy_config_into(JE->rob,qc,&qcm); 
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
      if((ct->rlgPt != NULL) &&  (ct->rlgPt->rlgchPt != NULL))
	J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
      else
	J = JE->rob->joints[1];
      ljnt = 0.0;
      naj = 0;
      while(J != ct->pasjnts[0]) {
	naj++;
	ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
	//printf("ljnt_iter = %f\n",ljnt);
	// WARNING : MAYBE NEXT JOINT TO BE TREATED IS NOT jntPt->next_jnt[0] !!!
	J = J->next_jnt[0];
      }
      aqdist = sqrt(ljnt);
      //printf("aqdist = %f\n",aqdist);
      dlfact = 100.0;   // ???!!!
      p3d_destroy_config(JE->rob,qc); 
      p3d_destroy_config(JE->rob,qcm); 

      // **************************************
      //IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
      //           y utilizarlo en este test
      // **************************************
      // NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
      //printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
      //if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ???? 
      if((minpqdist) < (dlfact * aqdist)) {
	//printf("  -> OK\n");
	for(i=0; i<6; i++) {
	  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i]);
	}
	nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
      }
      else {
	//printf("  -> FAIL\n");
	nvalidsol = 0;
      }
    }
  }
   
  if(nvalidsol == 0) {
    for(i=0; i<6; i++) {
      p3d_jnt_set_dof(ct->pasjnts[i],0, qlast[i]);
    }
  }
    
  for(i = 0; i < nsol; i++) {
    free(valid_sol_configs[i]);
  }
  free(valid_sol_configs);
  
  for(i = 0; i < 16; i++) {
    free(sol_configs[i]);
  }
  free(sol_configs);

  // local iksol
  st_iksol[ct->num] = nvalidsol; 

  return (nvalidsol);
/*   return (TRUE); */
}


/*****************************************************/

/* -- functions for 6R_bio_ik case of peptide bonds at 180 degrees -- */

/* WARNING : this function must be called with the peptide angles set at 180 */
static int p3d_set_6R_bio_ik_nopep(p3d_cntrt_management * cntrt_manager,
				   p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				   p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				   int ct_num, int state)
{ 
  p3d_jnt *jnt_prevPt, *jntPt;
  p3d_cntrt *ct;
  p3d_jnt * act_jnt_arrPt[6];
  int act_jnt_dof_arrPt[6];
  int act_rob_dof_arrPt[6];
  p3d_jnt * pas_jnt_arrPt[9];
  int pas_jnt_dof_arrPt[9];
  int pas_rob_dof_arrPt[9];
  int i;
  p3d_vector3 posJf,posJl,pos_diff;

  p3d_rob *r;
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* set passive joints */
  jntPt = jnt_prevPt = pas_jntPt[0];
  for(i=0; i<9; i++) {
    if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || 
	(jntPt->type != P3D_ROTATE))
      { return FALSE; }
    pas_jnt_arrPt[i] = jntPt;
    pas_jnt_dof_arrPt[i] = 0;
    pas_rob_dof_arrPt[i] = pas_rob_dof[0] +
      jntPt->index_dof - pas_jnt_arrPt[0]->index_dof;
    
    jnt_prevPt = jntPt;
    if(jntPt->next_jnt == NULL)
      jntPt = NULL;
    else
      jntPt = jntPt->next_jnt[jntPt->n_next_jnt - 1];
  } 

  /*set active joints */
  if(act_jntPt[0]->type != P3D_FREEFLYER) {
    PrintError(("p3d_set_6R_bio_ik_nopep : act_jnt must be a P3D_FREEFLYER"));
    return FALSE; 
  } 
  for(i=0; i<6; i++) {
    act_jnt_arrPt[i] = act_jntPt[0];
    act_jnt_dof_arrPt[i] = i;
    act_rob_dof_arrPt[i] = act_jntPt[0]->index_dof + i;
    cntrt_manager->in_cntrt[act_rob_dof_arrPt[i]] = DOF_ACTIF;
  }

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_6R_BIO_IK_NAME_NOPEP, 
				   9, pas_jnt_arrPt, pas_jnt_dof_arrPt, pas_rob_dof_arrPt,
				   6, act_jnt_arrPt, act_jnt_dof_arrPt, act_rob_dof_arrPt);
    if (ct == NULL)
      { return FALSE; }

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

    if(!bio_set_ik_nopep(ct))
      { return FALSE; }

    // reshoot for this constraint (when there are several loops or several cntrts in the loop)
    if(cntrt_manager->ncntrts > 1) {
      ct->reshoot_ct = ct;
    }
    ct->nctud = 1;
    ct->ct_to_update = MY_ALLOC(pp3d_cntrt, 1);
    ct->ct_to_update[0] = ct;

    // max. extension 
    // consider max. extension when all jnt are at 180
    // WARNING : consider the the chain is now set whit such a configuration
    p3d_jnt_get_cur_vect_point(ct->pasjnts[0],posJf);
    p3d_jnt_get_cur_vect_point(ct->pasjnts[8],posJl);
    p3d_vectSub(posJf,posJl,pos_diff);
    ct->argu_d[MAX_ARGU_CNTRT - 1] = (double) p3d_vectNorm(pos_diff);

  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}

static int p3d_fct_6R_bio_ik_nopep(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *JE,*J;
  double **sol_configs;
  double **valid_sol_configs;
  int i,inci;
  double min,max;
  double qlast[9];
  int nsol,nvalidsol,ns,imds;
  int fail;
  double pqdist,minpqdist,aqdist;
  configPt qc,qcm;
  double dlfact;
  double ljnt;
  int naj;
  
  // local iksol allocation
  if(st_iksol == NULL)
    alloc_and_init_st_iksol(ct->cntrt_manager);

  /* the chain contains 3 AA
     - the last joint (9) is fictive but required for refs
     - joints 3, 6 and 9 are the peptide bonds -> fixded at 180 degrees
  */
  p3d_jnt_get_dof_bounds(ct->pasjnts[2],0,&min,&max);
  if(max < M_PI)  
    p3d_jnt_set_dof(ct->pasjnts[2],0,-M_PI);
  else
    p3d_jnt_set_dof(ct->pasjnts[2],0,M_PI);
  p3d_jnt_get_dof_bounds(ct->pasjnts[5],0,&min,&max);
  if(max < M_PI)  
    p3d_jnt_set_dof(ct->pasjnts[5],0,-M_PI);
  else
    p3d_jnt_set_dof(ct->pasjnts[5],0,M_PI);
  p3d_jnt_get_dof_bounds(ct->pasjnts[8],0,&min,&max);
  if(min == max) {
    p3d_jnt_set_dof(ct->pasjnts[8],0,ct->pasjnts[8]->v);
  }
  else {
    if(max < M_PI)  
      p3d_jnt_set_dof(ct->pasjnts[8],0,-M_PI);
    else
      p3d_jnt_set_dof(ct->pasjnts[8],0,M_PI);
  }

  JE = ct->actjnts[0];

  p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);

  // PRINT LOOP GEOMETRY
  // (only if the active jnt has been modified)
  if(bio_get_PRINT_LOOP_GEOM() &&
     p3d_go_into_cntrt_fct(ct) &&
     (!p3d_jnt_get_dof_is_modified(JE->rob->joints[1],0))) {
    bio_print_loop_geometry(ct);
  }

  sol_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }
  
  /* compute all ik solutions */
  nsol = bio_compute_ik_nopep(ct,sol_configs);
  if(!nsol) {
    for(i = 0; i < 16; i++) {
      free(sol_configs[i]);
    }
    free(sol_configs);
    return(FALSE);
/*     return(TRUE); */
  }

  for(i=0; i<9; i++) {
    qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
  }  

  valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
  for(i = 0; i < nsol; i++) {
    valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  

  ns = 0;
  nvalidsol = 0;
  while(ns<nsol) {
    /*     printf("\n"); */
    fail = 0;
    inci = 0;
    for(i=0; (i<8) && (!fail); i++) {
      if((i == 2)||(i == 5)) {
	inci++;
      }
      else {
	p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &min, &max);
	if((sol_configs[ns][i-inci] <= max)&&(sol_configs[ns][i-inci] >= min)) {
	  valid_sol_configs[nvalidsol][i-inci] = sol_configs[ns][i-inci];
	  /* 	  printf(" %f ",sol_configs[ns][i-inci]); */
	  if(i == 7)
	    nvalidsol++;
	}
	else {
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
  if(look_iksol)
    iksol = st_iksol[ct->num];
  
  if(nvalidsol > 0) {
    if(qp == NULL) {
      if(iksol != -1) {
	/* if specified iksol */
	ns = iksol - 1;
      }
      else {
	/* choice a solution at random */
	ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6));
      }
      inci = 0;
      for(i=0; i<8 ; i++) {
	if((i == 2)||(i == 5)) {
	  inci++;
	}
	else {
	  p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i-inci]);
	}
      }       
      nvalidsol = ns + 1;
    }
    else {
      /* choice solution depending on qp */
      /* NOTE : the test of distance is made with the whole conf.
	        this could be a problem when the system has high n dofs
		-> the test should be made only for the passive dofs !!!
      */
      minpqdist = P3D_HUGE;
      qc = p3d_alloc_config(JE->rob);
      qcm = p3d_alloc_config(JE->rob);
      imds = 0;
      for(ns=0; ns<nvalidsol; ns++) {
	inci = 0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i-inci]);
	  }
	}       	
	p3d_get_robot_config_into(JE->rob, &qc);
	if(DEBUG_CNTRTS) {
	  // visualizar qp y q
	  p3d_set_robot_config(JE->rob,qp);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	  p3d_set_robot_config(JE->rob,qc);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	}
	ljnt = 0.0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc, qp));
	    //printf("ljnt_iter = %f\n",ljnt);
	  }
	}
	pqdist = sqrt(ljnt);
	//printf("pqdist_i = %f\n",pqdist);
/* 	qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
	if(pqdist < minpqdist) {
	  minpqdist = pqdist;
	  imds = ns;
	  p3d_copy_config_into(JE->rob,qc,&qcm); 
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
      if((ct->rlgPt != NULL) &&  (ct->rlgPt->rlgchPt != NULL))
	J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
      else
	J = JE->rob->joints[1];
      ljnt = 0.0;
      naj = 0;
      while(J != ct->pasjnts[0]) {
	naj++;
	ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
	//printf("ljnt_iter = %f\n",ljnt);
	J = J->next_jnt[J->n_next_jnt - 1];
      }
      aqdist = sqrt(ljnt);
      //printf("aqdist = %f\n",aqdist);
      dlfact = 100.0;   // ???!!!
      p3d_destroy_config(JE->rob,qc); 
      p3d_destroy_config(JE->rob,qcm); 

      // **************************************
      //IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
      //           y utilizarlo en este test
      // **************************************
      // NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
      //printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
      //if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ???? 
      //if((minpqdist) < (dlfact * aqdist)) {
      if((minpqdist) < 10.0) {
	//printf("  -> OK\n");
	inci = 0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i-inci]);
	  }
	}
	nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
      }
      else {
	//printf("  -> FAIL\n");
	nvalidsol = 0;
      }
    }
  }
   
  if(nvalidsol == 0) {
    for(i=0; i<9; i++) {
      p3d_jnt_set_dof(ct->pasjnts[i],0, qlast[i]);
    }
  }
    
  for(i = 0; i < nsol; i++) {
    free(valid_sol_configs[i]);
  }
  free(valid_sol_configs);
  
  for(i = 0; i < 16; i++) {
    free(sol_configs[i]);
  }
  free(sol_configs);

  // local iksol
  st_iksol[ct->num] = nvalidsol; 

  return (nvalidsol);
/*   return (TRUE); */
}

////////////////////////////////////////////////////////////////////////////////////

/* WARNING : this function must be called with the peptide angles set at 180 */
static int p3d_set_6R_bio_ik_nopep_new(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       double *Dval, int ct_num, int state)
{ 
  p3d_jnt *jnt_prevPt, *jntPt;
  p3d_cntrt *ct;
  p3d_jnt * act_jnt_arrPt[6];
  int act_jnt_dof_arrPt[6];
  int act_rob_dof_arrPt[6];
  p3d_jnt * pas_jnt_arrPt[9];
  int pas_jnt_dof_arrPt[9];
  int pas_rob_dof_arrPt[9];
  int i;
  p3d_vector3 posJf,posJl,pos_diff;

  p3d_rob *r;
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* set passive joints */
  jntPt = jnt_prevPt = pas_jntPt[0];
  for(i=0; i<9; i++) {
    if ((jntPt == NULL) || (jntPt->rob != jnt_prevPt->rob) || 
	(jntPt->type != P3D_ROTATE))
      { return FALSE; }
    pas_jnt_arrPt[i] = jntPt;
    pas_jnt_dof_arrPt[i] = 0;
    pas_rob_dof_arrPt[i] = pas_rob_dof[0] +
      jntPt->index_dof - pas_jnt_arrPt[0]->index_dof;
    
    jnt_prevPt = jntPt;
    if(jntPt->next_jnt == NULL)
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
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_6R_BIO_IK_NAME_NOPEP_NEW, 
				   9, pas_jnt_arrPt, pas_jnt_dof_arrPt, pas_rob_dof_arrPt,
                                   //6, act_jnt_arrPt, act_jnt_dof_arrPt, act_rob_dof_arrPt);
				   1, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }

    ct->fct_cntrt = p3d_fct_6R_bio_ik_nopep_new;
    /* initialize the parameters number */
    ct->ndval = 12;
    ct->nival = 0;

    for(i=0; i<12; i++)
      ct->argu_d[i] = Dval[i];

    if(!bio_set_ik_nopep(ct))
      { return FALSE; }

    // reshoot for this constraint (when there are several loops or several cntrts in the loop)
    if(cntrt_manager->ncntrts > 1) {
      ct->reshoot_ct = ct;
    }
    ct->nctud = 1;
    ct->ct_to_update = MY_ALLOC(pp3d_cntrt, 1);
    ct->ct_to_update[0] = ct;

    // max. extension 
    // consider max. extension when all jnt are at 180
    // WARNING : consider the the chain is now set whit such a configuration
    p3d_jnt_get_cur_vect_point(ct->pasjnts[0],posJf);
    p3d_jnt_get_cur_vect_point(ct->pasjnts[8],posJl);
    p3d_vectSub(posJf,posJl,pos_diff);
    ct->argu_d[MAX_ARGU_CNTRT - 1] = (double) p3d_vectNorm(pos_diff);

  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_6R_bio_ik_nopep_new(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *JE,*J;
  double **sol_configs;
  double **valid_sol_configs;
  int i,inci;
  double min,max;
  double qlast[9];
  int nsol,nvalidsol,ns,imds;
  int fail;
  double pqdist,minpqdist,aqdist;
  configPt qc,qcm;
  double dlfact;
  double ljnt;
  int naj;

  //if(p3d_go_into_cntrt_fct(ct)) {
  
    // local iksol allocation
  if(st_iksol == NULL)
    alloc_and_init_st_iksol(ct->cntrt_manager);
  
  /* the chain contains 3 AA
     - the last joint (9) is fictive but required for refs
     - joints 3, 6 and 9 are the peptide bonds -> fixded at 180 degrees
  */
  p3d_jnt_get_dof_bounds(ct->pasjnts[2],0,&min,&max);
  if(max < M_PI)  
    p3d_jnt_set_dof(ct->pasjnts[2],0,-M_PI);
  else
    p3d_jnt_set_dof(ct->pasjnts[2],0,M_PI);
  p3d_jnt_get_dof_bounds(ct->pasjnts[5],0,&min,&max);
  if(max < M_PI)  
    p3d_jnt_set_dof(ct->pasjnts[5],0,-M_PI);
  else
    p3d_jnt_set_dof(ct->pasjnts[5],0,M_PI);
  p3d_jnt_get_dof_bounds(ct->pasjnts[8],0,&min,&max);
  if(min == max) {
    p3d_jnt_set_dof(ct->pasjnts[8],0,ct->pasjnts[8]->v);
  }
  else {
    if(max < M_PI)  
      p3d_jnt_set_dof(ct->pasjnts[8],0,-M_PI);
    else
      p3d_jnt_set_dof(ct->pasjnts[8],0,M_PI);
  }
  
  JE = ct->actjnts[0];
  
  p3d_update_this_robot_pos_without_cntrt_and_obj(JE->rob);
  
  sol_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }
  
  /* compute all ik solutions */
  nsol = bio_compute_ik_nopep_new(ct,sol_configs);
  if(!nsol) {
    for(i = 0; i < 16; i++) {
      free(sol_configs[i]);
    }
    free(sol_configs);
    return(FALSE);
    //return(TRUE);
  }
  
  for(i=0; i<9; i++) {
    qlast[i] = p3d_jnt_get_dof(ct->pasjnts[i], 0);
  }  
  
  valid_sol_configs = (double **)malloc(sizeof(double *) * nsol);
  for(i = 0; i < nsol; i++) {
    valid_sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  
  
  ns = 0;
  nvalidsol = 0;
  while(ns<nsol) {
    /*     printf("\n"); */
    fail = 0;
    inci = 0;
    for(i=0; (i<8) && (!fail); i++) {
      if((i == 2)||(i == 5)) {
	inci++;
      }
      else {
	p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &min, &max);
	// SETTING INTO MIN_MAX RANGE ------
	if((max > M_PI) && (sol_configs[ns][i-inci] < min))
	  sol_configs[ns][i-inci] += (2.0 * M_PI);
	if((min < -M_PI) && (sol_configs[ns][i-inci] > max))
	  sol_configs[ns][i-inci] -= (2.0 * M_PI);
	//----------------------------------
	if((sol_configs[ns][i-inci] <= max)&&(sol_configs[ns][i-inci] >= min)) {
	  valid_sol_configs[nvalidsol][i-inci] = sol_configs[ns][i-inci];
	  /* 	  printf(" %f ",sol_configs[ns][i-inci]); */
	  if(i == 7)
	      nvalidsol++;
	}
	else {
	  fail = 1;
	} 
      }
    }  
    /*     printf("\n"); */
    ns++;
  }

  if(nvalidsol == 0) {
    //printf("*** All IK solutions out of jnt bounds ***\n");
  }
  
  // set st_niksol
  st_niksol[ct->num] = nvalidsol;
  
  /* WARNING :
     iksol is not an argument !!!
     it is get from the vector (which has been modified)
  */
  if(look_iksol)
    iksol = st_iksol[ct->num];
  
  if(nvalidsol > 0) {
    if(qp == NULL) {
      if(iksol != -1) {
	/* if specified iksol */
	ns = iksol - 1;
      }
      else {
	/* choice a solution at random */
	//ns = (int)floor(p3d_random(0.0,(double)nvalidsol-EPS6));
	/* choice the closest configuration to qlast */
	minpqdist = P3D_HUGE;
	imds = 0;
	for(ns=0; ns<nvalidsol; ns++) {
	  inci = 0;
	  ljnt = 0.0;
	  for(i=0; i<8; i++) {
	    if((i == 2)||(i == 5)) {
	      inci++;
	    }
	    else {
	      ljnt += SQR(valid_sol_configs[ns][i-inci] - qlast[i]);
	      //printf("ljnt_iter = %f\n",ljnt);
	    }
	  }
	  pqdist = sqrt(ljnt);
	  //printf("pqdist_i = %f\n",pqdist);
	  /* 	qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
	  if(pqdist < minpqdist) {
	    minpqdist = pqdist;
	    imds = ns;
	  }
	}
	
	dlfact = 1.0;   // ???!!!
	
	if((minpqdist) < (dlfact)) {
	  //printf("  -> OK\n");
	  inci = 0;
	  for(i=0; i<8; i++) {
	    if((i == 2)||(i == 5)) {
	      inci++;
	    }
	    else {
	      p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i-inci]);
	    }
	  }
	  nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
	}
	else {
	  //printf("  -> FAIL\n");
	  nvalidsol = 0;
	}
      }
    }
    else {
      /* choice solution depending on qp */
      /* NOTE : the test of distance is made with the whole conf.
	 this could be a problem when the system has high n dofs
	 -> the test should be made only for the passive dofs !!!
      */
      minpqdist = P3D_HUGE;
      qc = p3d_alloc_config(JE->rob);
      qcm = p3d_alloc_config(JE->rob);
      imds = 0;
      for(ns=0; ns<nvalidsol; ns++) {
	inci = 0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[ns][i-inci]);
	  }
	}       	
	p3d_get_robot_config_into(JE->rob, &qc);
	if(DEBUG_CNTRTS) {
	  // visualizar qp y q
	  p3d_set_robot_config(JE->rob,qp);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	  p3d_set_robot_config(JE->rob,qc);
	  p3d_update_this_robot_pos_without_cntrt(JE->rob);
	  g3d_draw_allwin_active();
	}
	ljnt = 0.0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    ljnt += SQR(p3d_jnt_calc_dof_dist(ct->pasjnts[i], 0, qc, qp));
	    //printf("ljnt_iter = %f\n",ljnt);
	  }
	}
	pqdist = sqrt(ljnt);
	//printf("pqdist_i = %f\n",pqdist);
	/* 	qdist = p3d_dist_q1_q2(JE->rob,qp,qc); */
	if(pqdist < minpqdist) {
	  minpqdist = pqdist;
	  imds = ns;
	  p3d_copy_config_into(JE->rob,qc,&qcm); 
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
      if((ct->rlgPt != NULL) &&  (ct->rlgPt->rlgchPt != NULL))
	J = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
      else
	J = JE->rob->joints[1];
      ljnt = 0.0;
      naj = 0;
      while(J != ct->pasjnts[0]) {
	naj++;
	ljnt += SQR(p3d_jnt_calc_dof_dist(J, 0, qcm, qp));
	//printf("ljnt_iter = %f\n",ljnt);
	J = J->next_jnt[J->n_next_jnt - 1];
      }
      aqdist = sqrt(ljnt);
      //printf("aqdist = %f\n",aqdist);
      dlfact = 100.0;   // ???!!!
      p3d_destroy_config(JE->rob,qc); 
      p3d_destroy_config(JE->rob,qcm); 
      
      // **************************************
      //IDEA !!! : puedo guardar en una variable el factor minpqdist/aqdist del test anterior
      //           y utilizarlo en este test
      // **************************************
      // NOTA : he constatado que minpqdist suele ser entre 5 y 6 veces mayor que aqdist -> PORQUE ???
      //printf("minpqdist = %f, aqdist = %f, npasj = %d, nactj = %d",minpqdist,aqdist,ct->npasjnts,naj);
      //if((minpqdist / ct->npasjnts) < (dlfact * (aqdist / naj))) {     // QUE PASA CON DISTANCIAS ???? 
      //if((minpqdist) < (dlfact * aqdist)) {
      //printf("minpqdist = %f\n",minpqdist);
      if((p3d_equal_config(JE->rob,qp,JE->rob->ROBOT_POS))||  // PROBLEMA CON ALGUNOS EJEMPLOS : LA PRIMERA CONFIGURACION REQUIERE UN "SALTO" MAYOR
	 (minpqdist < 0.8)) {    // REGLAJE DELICADO : DEBERIA DE DEPENDER DE LA LONGITUD DEL STEP !!! (Juan)
	inci = 0;
	for(i=0; i<8; i++) {
	  if((i == 2)||(i == 5)) {
	    inci++;
	  }
	  else {
	    p3d_jnt_set_dof(ct->pasjnts[i],0,valid_sol_configs[imds][i-inci]);
	  }
	}
	nvalidsol = imds + 1;  // need to add 1 to avoid returning 0 !
      }
      else {
	//printf("  -> FAIL\n");
	nvalidsol = 0;
      }
    }
  }
  
  if(nvalidsol == 0) {
    for(i=0; i<9; i++) {
      p3d_jnt_set_dof(ct->pasjnts[i],0, qlast[i]);
    }
  }
  
  for(i = 0; i < nsol; i++) {
    free(valid_sol_configs[i]);
  }
  free(valid_sol_configs);
  
  for(i = 0; i < 16; i++) {
    free(sol_configs[i]);
  }
  free(sol_configs);
  
  // local iksol
  st_iksol[ct->num] = nvalidsol; 
  
  return (nvalidsol);
}

//  else 
//    return (TRUE);
//}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for BIO bkb Hbonds test -- */
static int p3d_set_bio_bkb_Hbond_cntrt(p3d_cntrt_management * cntrt_manager,
				       p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				       p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				       double *Dval, int *Ival, int ct_num, int state)
{ 
  p3d_cntrt *ct;
  p3d_rob *robPt;
  p3d_matrix4 invT;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_BIO_BKB_HBOND, 
				   0, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				   0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
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
    if(Ival[3] == 1) { // N-O bond
      p3d_mat4Copy(p3d_mat4IDENTITY,ct->Tbase);
      p3d_mat4Copy(robPt->joints[Ival[1]]->o->pol[1]->pos_rel_jnt,ct->Tatt);
    }
    else {             // O-N bond
      p3d_mat4Copy(robPt->joints[Ival[0]]->o->pol[1]->pos_rel_jnt,ct->Tbase);
      p3d_matInvertXform(robPt->joints[Ival[1]]->abs_pos,invT);
      // NOTE : joint associanted with C has only one next_jnt
      p3d_matMultXform(invT,robPt->joints[Ival[1]]->next_jnt[0]->abs_pos,ct->Tatt);
    }

    // enchain with main constraint of the loop
    if(Ival[2] >= 0) {
      p3d_add_to_cntrts_chain(cntrt_manager->cntrts[Ival[2]],ct,cntrt_manager->cntrts[Ival[2]]->act_rob_dof[0]);
    }

  } else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  ct->argu_d[0] = Dval[0];
  ct->argu_d[1] = Dval[1];

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_bio_bkb_Hbond_cntrt(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *Jend,*Jbase;
  p3d_matrix4 Tbase,Tend;
  p3d_vector3 posA1,posA2,pos_diff;
  double distance;

  // WARNING : this shold not be made : can give problems for multiple robots 
  p3d_rob *robPt;
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ////////////

  // WARNING : suppose that cofiguration has been set and updated !!!
/*   if(!p3d_get_RLG()) */
/*     p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ??? */

  Jbase = robPt->joints[ct->argu_i[0]];
  Jend  = robPt->joints[ct->argu_i[1]];

  p3d_mat4Mult(Jbase->abs_pos,ct->Tbase,Tbase);
  p3d_mat4Mult(Jend->abs_pos,ct->Tatt,Tend);

  posA1[0] = Tbase[0][3]; posA1[1] = Tbase[1][3]; posA1[2] = Tbase[2][3]; 
  posA2[0] = Tend[0][3]; posA2[1] = Tend[1][3]; posA2[2] = Tend[2][3]; 
  p3d_vectSub(posA2,posA1,pos_diff);	
  distance = (double) p3d_vectNorm(pos_diff);

  //printf("ctnum : %d  , distance : %f\n",ct->num,distance);
  
  if((distance < ct->argu_d[0]) || (distance > ct->argu_d[1])) {
    return(FALSE);
  }

  return TRUE;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* -- functions for BIO diS bonds test -- */
static int p3d_set_bio_diS_bond_cntrt(p3d_cntrt_management * cntrt_manager,
				      p3d_jnt **pas_jntPt,int *pas_jnt_dof,int *pas_rob_dof,
				      p3d_jnt **act_jntPt,int *act_jnt_dof,int *act_rob_dof,
				      double *Dval, int *Ival, int ct_num, int state)
{ 
  p3d_cntrt *ct;
  p3d_rob *robPt;
  p3d_poly *polPt=NULL;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  if (ct_num < 0) {
    ct = p3d_create_generic_cntrts(cntrt_manager, CNTRT_BIO_DIS_BOND, 
				   0, pas_jntPt, pas_jnt_dof, pas_rob_dof,
				   0, act_jntPt, act_jnt_dof, act_rob_dof);
    if (ct == NULL)
      { return FALSE; }
    ct->fct_cntrt = p3d_fct_bio_diS_bond_cntrt;

    // indices of first and last joints
    ct->argu_i[0] = Ival[0]; // joint of the 1st S
    ct->argu_i[1] = Ival[1]; // joint of the 2nd S

    // compute Tatt and Tbase (dending on the kind of bond, N-O or O-N)
    // WARNING : suppose that S is 1st (if no H) or 3rd (if H) primitive in the object
    if(robPt->joints[Ival[0]]->o->np == 1) {
      polPt = robPt->joints[Ival[0]]->o->pol[0];
    }
    else if(robPt->joints[Ival[0]]->o->np == 3) {
      polPt = robPt->joints[Ival[0]]->o->pol[2];
    }
    else {
      printf("ERROR : p3d_set_bio_diS_bond_cntrt : wrong np in obj %d\n",
	     robPt->joints[Ival[0]]->o->num);
      return FALSE;
    }
    // Transf. fron jnt for 1st S is stored in Tbase 
    p3d_mat4Copy(polPt->pos_rel_jnt,ct->Tbase);

    if(robPt->joints[Ival[1]]->o->np == 1) {
      polPt = robPt->joints[Ival[1]]->o->pol[0];
    }
    else if(robPt->joints[Ival[1]]->o->np == 3) {
      polPt = robPt->joints[Ival[1]]->o->pol[2];
    }
    else {
      printf("ERROR : p3d_set_bio_diS_bond_cntrt : wrong np in obj %d\n",
	     robPt->joints[Ival[1]]->o->num);
      return FALSE;
    }
    // Transf. fron jnt for 2nd S is stored in Tatt 
    p3d_mat4Copy(polPt->pos_rel_jnt,ct->Tatt);
  } 
  else {
    ct = cntrt_manager->cntrts[ct_num];
  }    

  ct->argu_d[0] = Dval[0];
  ct->argu_d[1] = Dval[1];

  if ((!state) || (!(ct->active) && state)) {
    if (!p3d_update_jnts_state(ct, state))
      { return FALSE; }
  }
  ct->active = state;
  last_cntrt_set = ct;
  return(TRUE);
}


static int p3d_fct_bio_diS_bond_cntrt(p3d_cntrt *ct, int iksol, configPt qp, double dl)
{
  p3d_jnt *Jend,*Jbase;
  p3d_matrix4 Tbase,Tend;
  p3d_vector3 posA1,posA2,pos_diff;
  double distance;

  // WARNING : this shold not be made : can give problems for multiple robots 
  p3d_rob *robPt;
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ////////////

  if(!p3d_get_RLG())
    p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); // necessary ???

  Jbase = robPt->joints[ct->argu_i[0]];
  Jend  = robPt->joints[ct->argu_i[1]];

  p3d_mat4Mult(Jbase->abs_pos,ct->Tbase,Tbase);
  p3d_mat4Mult(Jend->abs_pos,ct->Tatt,Tend);

  posA1[0] = Tbase[0][3]; posA1[1] = Tbase[1][3]; posA1[2] = Tbase[2][3]; 
  posA2[0] = Tend[0][3]; posA2[1] = Tend[1][3]; posA2[2] = Tend[2][3]; 
  p3d_vectSub(posA2,posA1,pos_diff);	
  distance = (double) p3d_vectNorm(pos_diff);

  //printf("ctnum : %d  , distance : %f\n",ct->num,distance);
  
  if((distance < ct->argu_d[0]) || (distance > ct->argu_d[1])) {
    return(FALSE);
  }

  return TRUE;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/* function returning if there are active cntrts or not */

int p3d_actived_cntrts(p3d_cntrt_management *cntrt_manager)
{
  p3d_cntrt *ct;

  if((cntrt_manager == NULL)||(cntrt_manager->cntrts == NULL)) {
    return FALSE;
  }
  else {
    for (dbl_list_goto_first(cntrt_manager->cntrt_call_list);
	 dbl_list_more(cntrt_manager->cntrt_call_list); 
	 dbl_list_next(cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, cntrt_manager->cntrt_call_list);
      if(ct->active)
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

int p3d_check_cntrts_at_conf(p3d_rob *robotPt, configPt q)
{
  p3d_cntrt *ct;

  p3d_set_robot_config(robotPt, q);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);

  /* change la configuration s'il y a des contraintes cinematiques */
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	 dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	 dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active)
	if(!(*ct->fct_cntrt)(ct,-1,NULL,0.0)) 
	  return FALSE;
    }
  }

  return TRUE;
}


int p3d_check_cntrts_at_conf_multisol(p3d_rob *robotPt, configPt q, configPt qp, double dl)
{
  p3d_cntrt *ct;

  p3d_set_robot_config(robotPt, q);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);

  /* change la configuration s'il y a des contraintes cinematiques */
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	 dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	 dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active)
	if(!(*ct->fct_cntrt)(ct,-1,qp,dl)) 
	  return FALSE;
    }
  }

  return TRUE;
}



/*---------------------------------------------------------------------------*/


int p3d_cntrt_localpath_classic_test(p3d_rob *robotPt,
				     p3d_localpath *localpathPt,
				     double *Kpath)
{
  double u, du, umax;
  double dmax; 
  int valid = 1;
  int end_localpath = 0;
  double *distances;
  int j;
  int njnt = robotPt->njoints;
  configPt qsave;

  if (localpathPt == NULL)
    { return FALSE; }

  /* Some curves can be decided unvalid by the user */
  if (localpathPt->valid == FALSE)
    { return FALSE; }  

  if(p3d_actived_cntrts(robotPt->cntrt_manager)) {
    *Kpath = 0.0;
    umax = localpathPt->range_param;
    distances = MY_ALLOC(double, njnt+1);
    /* NOTE :
       we have chosen same discretization step than in collision checking
       however it can be different !!!
    */
    p3d_col_get_dmax(&dmax);
    dmax *= 10;
    
    /* current position of robot is saved */
    qsave = p3d_get_robot_config(robotPt);

    for (j=0; j<=njnt; j++)
      { distances[j] = dmax; }    
    u = 0.0;
    du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
				       FORWARD, distances);
    
    u = du;
    if (u > umax - EPS6) {      
      end_localpath = 1;
    }

    while(!end_localpath) {
      /* position of the robot corresponding to parameter u */
      if (change_position_robot_without_obj(robotPt, localpathPt, u)) {
	p3d_set_and_update_this_robot_conf(robotPt, qsave);
	p3d_destroy_config(robotPt, qsave);
	MY_FREE(distances, double, njnt+1);
	return FALSE;
      }
      
      *Kpath = u / localpathPt->range_param;

      for (j=0; j<=njnt; j++)
	{ distances[j] += dmax; }

      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);    
      u+=du;
      if (u > umax - EPS6){
	end_localpath++;
      }
    }
    p3d_set_and_update_this_robot_conf(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    MY_FREE(distances, double, njnt+1);
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
static void alloc_and_init_st_iksol(p3d_cntrt_management *cntrt_manager)
{  
  int i;

  st_iksol = MY_ALLOC(int, cntrt_manager->ncntrts);
  st_niksol = MY_ALLOC(int, cntrt_manager->ncntrts);
  for(i=0; i<cntrt_manager->ncntrts; i++) {
    st_iksol[i] = -1;
    st_niksol[i] = -1;
  }
}


void p3d_get_iksol_vector(p3d_cntrt_management *cntrt_manager, int *iksol)
{
  int i;

  if(st_iksol == NULL) {
    iksol = NULL;
  }
  else {
    if(iksol == NULL)
      // OJO !!!
      // esto no funciona (hay que usar **)
      // o hacer el alloc en la funcion que llama a esta (como para p3d_get_niksol_vector)
      iksol = MY_ALLOC(int, cntrt_manager->ncntrts);
    for(i=0; i<cntrt_manager->ncntrts; i++)
      iksol[i] = st_iksol[i];
  }
}


int p3d_is_multisol(void)
{
  if(st_niksol != NULL)
    return 1;
  else
    return 0;
}


void p3d_get_niksol_vector(p3d_cntrt_management *cntrt_manager, int *niksol)
{
  int i;

  if(niksol != NULL) {
    for(i=0; i<cntrt_manager->ncntrts; i++)
      niksol[i] = st_niksol[i];
  }
}

void p3d_set_iksol_elem(int i, int val)
{
  if(st_iksol != NULL) {
    st_iksol[i] = val;
  }
}

// void p3d_set_look_iksol(int val)
// {
//   look_iksol = val;
// }

void p3d_copy_iksol(p3d_cntrt_management *cntrt_manager, int *iksol_src, int *iksol_dst)
{
  int i;

  if(iksol_src == NULL) {
    iksol_dst = NULL;
  }
  else {
    iksol_dst = MY_ALLOC(int, cntrt_manager->ncntrts);
    for(i=0; i<cntrt_manager->ncntrts; i++)
      iksol_dst[i] = iksol_src[i];
  }
}

void p3d_destroy_iksol(p3d_cntrt_management *cntrt_manager, int *iksol)
{
  MY_FREE(iksol, int, cntrt_manager->ncntrts);
}

void p3d_destroy_niksol(p3d_cntrt_management *cntrt_manager, int *niksol)
{
  MY_FREE(niksol, int, cntrt_manager->ncntrts);
}


/* PROBLEM WITH PROTOS ???!!!:
   (next lines must be added 'by-hand' in p3d_constraints_proto.h 

extern int p3d_actived_cntrts(p3d_cntrt_management *cntrt_manager);
extern int p3d_check_cntrts_at_conf(p3d_rob *robotPt, configPt q);
extern int p3d_check_cntrts_at_conf_multisol(p3d_rob *robotPt, configPt q, configPt qp, double dl);
extern int p3d_cntrt_localpath_classic_test(p3d_rob *robotPt,
					    p3d_localpath *localpathPt,
					    double *Kpath);
extern void p3d_get_iksol_vector(p3d_cntrt_management *cntrt_manager, int *iksol);
extern int p3d_is_multisol(void);
extern void p3d_get_niksol_vector(p3d_cntrt_management *cntrt_manager, int *niksol);
extern void p3d_set_iksol_elem(int i, int val);
extern void p3d_set_look_iksol(int val);
extern void p3d_copy_iksol(p3d_cntrt_management *cntrt_manager, int *iksol_src, int *iksol_dst);
extern void p3d_destroy_iksol(p3d_cntrt_management *cntrt_manager, int *iksol);
extern void p3d_destroy_niksol(p3d_cntrt_management *cntrt_manager, int *niksol);

*/

/**********************************************************/
int p3d_local_conf_correction(p3d_rob *robotPt, configPt q)
{
  double val;
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax, vari;
  double dvmin, dvmax;
  p3d_jnt * jntPt;
  int ntry;
  configPt qp;
  double dl = 0.0;     // WARNING : unused by current cntrt functions

  qp = p3d_alloc_config(robotPt);
  p3d_copy_config_into(robotPt,q,&qp);  

  // try to generate valid conformation a number of times
  for(ntry=0;ntry<1000;ntry++) {
    // shoot 
    for(i=0; i<=njnt; i++) {
      jntPt = robotPt->joints[i];
	for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	  k = jntPt->index_dof + j;
	  if (p3d_jnt_get_dof_is_user(jntPt, j)
	      // NEW : DO NOT PERTURB FREEFLYING JNTS
	      && jntPt->type != P3D_FREEFLYER) {	    
	    p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	    val = p3d_jnt_get_dof(jntPt, j);
	    // allowed variation = +-10%
	    //vari = (vmax - vmin) / 20; 
	    vari = (vmax - vmin) * (double)ntry / (1000.0 * 10.0); 
	    dvmax = val + vari;
	    dvmin = val - vari;
	    if(dvmax > vmax)
	      dvmax = vmax;
	    if(dvmin < vmin)
	      dvmin = vmin;
	    q[k] = p3d_random(dvmin, dvmax);
	  } 
	  else {
	    q[k] = p3d_jnt_get_dof(jntPt, j);
	  }
	}	
    }
    if(p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,dl)) {
      p3d_destroy_config(robotPt,qp);
      return(TRUE);
    }
  }
  p3d_destroy_config(robotPt,qp);
  return(FALSE);
}
/**********************************************************/

