/* fichier p3d_hilflat.c */

/*  fichier �crit par Elodie-Jane Sims                     */
/*  en cas de probl�me ou si vous voulez des explications  */
/*  et que Florent n'est pas l� �crivez-moi : sims@ens.fr  */

/* tout ce qui commence par d est une distance et par u est un parametre */
/*tout ce qui est p3d_nom_func est la meme que nom_func mais avec des parametre de type p3d*/
/*for the function f_i is the i_th derivation of the function f*/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#define MAX_DERIV 4

#define JNT_BASE    0
#define JNT_CURV 1


static double v_cusp(p3d_rob *robotPt, configPt q1, configPt q2);
static double v_fcusp(p3d_rob *robotPt,
		      TR_FLAT_CONFIG_STR *q1, 
		      TR_FLAT_CONFIG_STR *q2);

static void acc_max(p3d_rob *robotPt, 
		    p3d_sub_hilflat_data * sub_hilflat_dataPt, 
		    double *tab_acc_max, double du);

static void unfree_x_y(p3d_jnt *jntPt, int num_theta, int * free_joints);

static void 
p3d_accelCoefficient(p3d_rob *robotPt, 
		     p3d_sub_hilflat_data *sub_hilflat_dataPt, 
				 double u, double *coeff);

static double dist_conf(p3d_rob *robotPt, configPt q1, configPt q2);


static double norme_gamma_1 (p3d_rob *robotPt, configPt q1, 
			     configPt q2, double u, 
			     double v2);

static double PAR_INIT = 0.;
static double PAR_END = 1.;

static double GAMMA_1_MIN_RANGE = 100.;

/**********************************************************************/
/* for memory this is the convertion of configPt and TR_CONFIG_STR */
/* because I use the conv_conf_fconf of general_flat.c to do this */
/**********************************************************************/
/*      TR_CONFIG_STR->xq = configPt[0]             */
/*      TR_CONFIG_STR->yq = configPt[1]             */
/*      conf->teta_1 = configPt[3]                  */
/*      conf->tau = configPt[3] + configPt[4]    */

/*      configPt[0] = TR_CONFIG_STR->xq             */
/*      configPt[1] = TR_CONFIG_STR->yq             */
/*      configPt[3] = conf->teta_1                  */
/*      configPt[4] = conf->tau - conf->teta_1   */
/**********************************************************************/
/* Conversion from ConfigPt to TR_FLAT_CONFIG_STR                     */
/*  l2: . distance between hilflat axis and hitch point               */
/**********************************************************************/
static void conv_conf_fconf(p3d_rob *robotPt, configPt q, 
			    TR_FLAT_CONFIG_STR *fconf)
{
  philflat_str hilflat_params = 
    lm_get_hilflat_lm_param(robotPt);

  int x_coord = hilflat_params->numdof[HILFLAT_DOF_X];
  int y_coord = hilflat_params->numdof[HILFLAT_DOF_Y];
  int theta_coord = hilflat_params->numdof[HILFLAT_DOF_THETA];
  int curvature_coord = hilflat_params->numdof[HILFLAT_DOF_CURV];

  double x = q[x_coord];
  double y = q[y_coord];
  double theta = q[theta_coord];
  double kappa = tan(q[curvature_coord]);
  
  fconf->xp = x;  
  fconf->yp = y;  
  fconf->tau = theta;
  fconf->kappa = kappa;
}
/**********************************************************************/
/* Conversion from TR_FLAT_CONFIG_STR to ConfigPt                     */
/**********************************************************************/
static void conv_fconf_conf(p3d_rob *robotPt, 
			    TR_FLAT_CONFIG_STR *fconf, 
			    configPt q)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  int j;

  int x_coord = hilflat_params->numdof[HILFLAT_DOF_X];
  int y_coord = hilflat_params->numdof[HILFLAT_DOF_Y];
  int theta_coord = hilflat_params->numdof[HILFLAT_DOF_THETA];
  int curvature_coord = hilflat_params->numdof[HILFLAT_DOF_CURV];
  
  /* set dof that are not used by hilflat local method to 0 */
  for (j = 0; j < robotPt->nb_dof; j++){
    q[j] = 0;
  }

  q[x_coord] = fconf->xp;
  q[y_coord] = fconf->yp;
  q[theta_coord] = fconf->tau;
  q[curvature_coord] = atan(fconf->kappa);
}

/**********************************************************************/
/*  Convert a p3d_sub_hilflat_data into a FLAT_LOCAL_PATH_STR */
/**********************************************************************/

void 
hilflat_conv_sub_path_fpath(p3d_rob *robotPt, 
			    p3d_sub_hilflat_data *sub_hilflat_dataPt, 
			    FLAT_LOCAL_PATH_STR *fpathPt)
{
  TR_FLAT_CONFIG_STR qf_init, qf_end;

  conv_conf_fconf(robotPt, sub_hilflat_dataPt->q_init, &qf_init);
  conv_conf_fconf(robotPt, sub_hilflat_dataPt->q_end, &qf_end);

  memcpy((void*)&(fpathPt->initFlatConf), (void*)&qf_init, 
	 sizeof(TR_FLAT_CONFIG_STR));
  memcpy((void*)&(fpathPt->finalFlatConf), (void*)&qf_end, 
	 sizeof(TR_FLAT_CONFIG_STR));

  fpathPt->v2 = sub_hilflat_dataPt->v;
  fpathPt->velCoeff = 1.0;
  fpathPt->u_start = sub_hilflat_dataPt->u_start;
  fpathPt->u_end = sub_hilflat_dataPt->u_end;
}


/**********************************************************************/
/*compute the configuration at a given parameter*/
/**********************************************************************/
static configPt p3d_combination(p3d_rob *robotPt, configPt q_init, 
				configPt q_end, double u, double v, 
				int deriv_order, double *Tab_gamma)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  configPt q = p3d_alloc_config(robotPt);
  p3d_jnt *jntPt;
  whichway backward;
  TR_FLAT_CONFIG_STR qf_init, qf_end, qf;
  int i, j;

  conv_conf_fconf (robotPt, q_init , &qf_init);
  conv_conf_fconf (robotPt, q_end , &qf_end);
  
  flatHilareCombination(&qf_init, &qf_end, u, v, 
			deriv_order, Tab_gamma);
  backward = ((v >=0) ? 0 : 1);
  flat_conv_curve_fconf(Tab_gamma, &qf, backward);
  conv_fconf_conf (robotPt, &qf, q);

  /* other dof are computed by linear interpolation */
  for (i = 0; i < hilflat_params->nb_other_jnt; i++){
    jntPt = robotPt->joints[hilflat_params->other_jnt[i]];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end,
						     u/(PAR_END - PAR_INIT));
    }
  }
  
  return q;
}

/**********************************************************************/
/*compute the configuration at a given parameter*/
/**********************************************************************/
static configPt 
p3d_Gamma(p3d_rob *robotPt, configPt qi, double u, double *Tab_gamma)
{
  configPt q = p3d_alloc_config(robotPt);
  p3d_jnt *jntPt;
  TR_FLAT_CONFIG_STR qf_i, qf;
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  int deriv_order = 3;
  int i, j;

  conv_conf_fconf (robotPt, qi, &qf_i);
  
  flatGamma(&qf_i, u, deriv_order, Tab_gamma);
  flat_conv_curve_fconf(Tab_gamma, &qf, 0.);
  conv_fconf_conf (robotPt, &qf, q);
  
  /* other joints are set to qi values */
  for (i = 0; i < hilflat_params->nb_other_jnt; i++){
    jntPt = robotPt->joints[hilflat_params->other_jnt[i]];
    for(j=jntPt->index_dof; j<(jntPt->index_dof+jntPt->dof_equiv_nbr); j++) { 
      q[j] = qi[j]; 
    }
  }
  
  return q;
}

/*****************************************************************************/
/**/
/*****************************************************************************/
static double p3d_V2(p3d_rob *robotPt, configPt q1, configPt q2){
  TR_FLAT_CONFIG_STR qf1, qf2;
  conv_conf_fconf(robotPt, q1, &qf1);
  conv_conf_fconf(robotPt, q2, &qf2);
  return flatV2(&qf1, &qf2);
}

/**********************************************************************/
/*return 1 if q1 and q2 are ordered else return -1*/
/*the order is in order the fabs of v2, the coordonates of the configurations*/
/**********************************************************************/
static whichway dir_order (p3d_rob *robotPt, configPt q1, configPt q2)
{
  double v_g, v_b;
  p3d_jnt *jntPt;
  philflat_str hilflat_params;
  int i, j;
  
  v_g = p3d_V2(robotPt, q1, q2);
  v_b = p3d_V2(robotPt, q2, q1);

  if (fabs(v_g) < fabs(v_b)){
    return -1;
  }
  if (fabs(v_g) > fabs(v_b)){
    return 1;
  }

  hilflat_params = lm_get_hilflat_lm_param(robotPt);
  for (i = 0; i < hilflat_params->nb_other_jnt; i++){
    jntPt = robotPt->joints[hilflat_params->other_jnt[i]];
    for(j=jntPt->index_dof; j<(jntPt->index_dof+jntPt->dof_equiv_nbr); j++) {
      if (q1[j] < q2[j]) {return 1;}
      if (q1[j] > q2[j]) {return -1;}
    }
  }

  return 1;
}

/**********************************************************************/
/*switch q1 and q2 if necessairy*/
/**********************************************************************/
static whichway order_conf_in_place (p3d_rob *robotPt, configPt *q1Pt, 
				     configPt *q2Pt)
{
  configPt q_switch;
  whichway dir = dir_order(robotPt, *q1Pt, *q2Pt);

  if (dir == -1){
    q_switch = (*q1Pt);
    (*q1Pt) = (*q2Pt);
    (*q2Pt) = q_switch;
  }
  return dir;
}

/*!
  In symmetric mode, any local planner has to be symeetric, that is 
  the path between q1 and q2 has to be the same as the path between
  q2 and q1. In its basic mode, the flatness-based local method are not 
  symmetric because of the cusp configuration. This function establishes 
  a total order between configurations and switches configurations in
  such a way that the local method is called between the smallest and the 
  biggest.
*/

static whichway order_conf (p3d_rob *robotPt, configPt q1, configPt q2, 
			    configPt*q_minPt, configPt *q_maxPt,
			    int symmetric)
{
  whichway dir=1;

  (*q_minPt) = q1;
  (*q_maxPt) = q2;
  if (symmetric == TRUE) {
    dir = order_conf_in_place (robotPt, q_minPt, q_maxPt);
  }
  
  return dir;
}

/*****************************************************************************/
/* Calcul de v2 l'abscisse curviligne sur le cercle Gamma1 de l'intersection */
/* de la droite O1M2 et Gamma_1                                              */
/*as V2 but start by ordering the configurations*/
/*****************************************************************************/
static double V2_order(p3d_rob *robotPt, configPt q1, configPt q2, 
		       int symmetric)
{
  double v2;
  whichway dir;

  configPt q_min;
  configPt q_max;
  dir = order_conf(robotPt, q1,q2,&q_min, &q_max, symmetric);
  v2 = dir * p3d_V2(robotPt, q_min, q_max);
 return v2;
}
/*****************************************************************************/
/**/
/*****************************************************************************/
static double p3d_V1(p3d_rob *robotPt, configPt q1, configPt q2){
  TR_FLAT_CONFIG_STR qf1, qf2;
  conv_conf_fconf(robotPt, q1, &qf1);
  conv_conf_fconf(robotPt, q2, &qf2);
  return flatV1(&qf1, &qf2);
}
/*****************************************************************************/
/*V1 compute the length between M2 and his projection on Gamma1 */
/*as V1 but start by ordering the configurations*/
/*****************************************************************************/
static double V1_order(p3d_rob *robotPt, configPt q1, configPt q2,
		       int symmetric)
{
  double v1;
  whichway dir;

  configPt q_min;
  configPt q_max;
  dir = order_conf(robotPt, q1, q2, &q_min, &q_max, symmetric);
  v1 = dir * p3d_V1(robotPt, q_min, q_max);
 return v1;
}

/*****************************************************************************/
/* the length is a kind of Pytagore of V1 and V2*/
/*****************************************************************************/
static double compute_length_traj2(p3d_rob *robotPt, configPt q1, configPt q2,
				   int symmetric)
{
  double lg;
  if ((q1==NULL)||(q2==NULL)){
    return 0;
  }
  lg = sqrt(pow(V2_order(robotPt, q1, q2, symmetric),2.0)+
	    pow(V1_order(robotPt, q1, q2, symmetric),2.0));
  return lg;
}


/**********************************************************************/
/* say if v is */
/* be careful k is a constante put here by random*/
/**********************************************************************/
static int is_v_valid(p3d_rob *robotPt,
		      p3d_sub_hilflat_data *sub_hilflat_dataPt,
		      int symmetric){
  double v = V2_order(robotPt, 
		      sub_hilflat_dataPt->q_init, 
		      sub_hilflat_dataPt->q_end, symmetric);
  double d = dist_conf(robotPt, 
		       sub_hilflat_dataPt->q_init, 
		       sub_hilflat_dataPt->q_end);
  double k = 0.01;
  return (fabs(v) >= k * d);
}

/*
 *  hilflat_face_to_face --
 *
 *  Test if two configuration are face to face. Such a pair of 
 *  configurations is dangerous for the local planner.
 *
 */

static int 
hilflat_face_to_face(p3d_rob *robotPt,
			 p3d_sub_hilflat_data *sub_hilflat_dataPt)
{

  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  int theta_coord = hilflat_params->numdof[HILFLAT_DOF_THETA];
  double theta_init = sub_hilflat_dataPt->q_init[theta_coord];
  double theta_end = sub_hilflat_dataPt->q_end[theta_coord];

  if (dist_circle(theta_init, theta_end) > .75*M_PI){
    return TRUE;
  }
  return FALSE;
}

/**********************************************************************/
/* say if gamma_1_min is valid */
/* be careful k is a constante put here by random*/
/**********************************************************************/
static int 
is_gamma_1_min_valid(p3d_rob *robotPt,
		     p3d_sub_hilflat_data *sub_hilflat_dataPt)
{
  double v = sub_hilflat_dataPt->v;
  double gamma_1_min = sub_hilflat_dataPt->gamma_1_min;
  double d;
  double k = 0.02;

  if (gamma_1_min < fabs(v)/GAMMA_1_MIN_RANGE) {
    return 0;
  }
  d = dist_conf(robotPt, 
		sub_hilflat_dataPt->q_init, 
		sub_hilflat_dataPt->q_end);
  return (gamma_1_min >= k * d);
}

/**********************************************************************/
/* say if we are in the case of close configuration */
/**********************************************************************/
static int are_close (p3d_rob *robotPt, configPt q1, configPt q2)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double length = robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;
  double d = dist_conf(robotPt, q1, q2);
  return (d < 2*length);
}

/**********************************************************************/
/* give a length between two configurations */
/*didn't care joints but the trail*/
/**********************************************************************/
static double dist_conf(p3d_rob *robotPt, configPt q1, configPt q2)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double res;
  TR_FLAT_CONFIG_STR qf_1, qf_2;
  double *coef = robotPt->length_array;
  double diam_robot = 
    robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;
  int i, j;
  p3d_jnt * jntPt;
  double ljnt = 0.;
  
  conv_conf_fconf(robotPt, q1, &qf_1);
  conv_conf_fconf(robotPt, q2, &qf_2);
  
  res = sqrt(pow(qf_1.xp - qf_2.xp,2.) + pow(qf_1.yp - qf_2.yp,2.)) +
    dist_circle(qf_1.tau, qf_2.tau)*diam_robot +
    fabs(qf_1.kappa - qf_2.kappa)*diam_robot*diam_robot;
  
  /* add distance computed for dof not used by hilflat local method */
  for (i = 0; i < hilflat_params->nb_other_jnt; i++){
    jntPt = robotPt->joints[hilflat_params->other_jnt[i]];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jntPt, j)) {
	ljnt += SQR(coef[j]*(q1[j+jntPt->index_dof]-q2[j+jntPt->index_dof]));
      } else {
	ljnt += SQR(q1[j+jntPt->index_dof]-q2[j+jntPt->index_dof]);
      }
    }
  }
  
  res += sqrt(ljnt);
  return res;
}

/***********************************************************************/
/* give a length between q2 and the cone of q1 */
/*where q1 and q2 are configPt*/
/***********************************************************************/
static double dist_cone (p3d_rob *robotPt, configPt q1, configPt q2){
  double v_cone = v_cusp(robotPt, q1, q2);
  double v2 = fabs(p3d_V2(robotPt, q1, q2));
  return (v_cone/v2);
}

/**********************************************************************/
/* allocation of a data structure specific to the hilflat local method */
/**********************************************************************/
static p3d_sub_hilflat_data *
p3d_alloc_spec_hilflat_sub_localpath(p3d_rob *robotPt,
				     configPt q_init, 
				     configPt q_end, 
				     double u_start, 
				     double u_end,
				     int symmetric)
{
  p3d_sub_hilflat_data * sub_hilflat_dataPt;
  double v;
  double tab_acc_max[8];
  double diam_rob;
  double new_u_start, new_u_end;

  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double du; /*the du for calculate the approximation of all the max*/
  static int acc_max_nb_step = 8;
  
  diam_rob = robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;

  sub_hilflat_dataPt = MY_ALLOC(p3d_sub_hilflat_data,1);
  if (sub_hilflat_dataPt == NULL) { 
    return NULL; 
  }

   if (u_start > u_end) { 
     return NULL; 
   }

  /*correction in case of u_start and u_end would not be in the path */
  new_u_start = MAX(u_start, PAR_INIT);
  new_u_end   = MAX(u_end,   PAR_INIT);
  new_u_start = MIN(u_start, PAR_END);
  new_u_end   = MIN(u_end,   PAR_END);

  sub_hilflat_dataPt->q_init = q_init;
  sub_hilflat_dataPt->q_end =  q_end;

  sub_hilflat_dataPt->u_start = new_u_start;
  sub_hilflat_dataPt->u_end = new_u_end;

  v = V2_order(robotPt, q_init, q_end, symmetric);
  sub_hilflat_dataPt->v = v;
/*   if (isf_v_valid(&qf_init, &qf_end)){ */
    /*here we should put the current way to compute the lenght of a trajectory*/
  sub_hilflat_dataPt->length = compute_length_traj2(robotPt, q_init, q_end, 
						    symmetric);
    
  du = (u_end-u_start)/acc_max_nb_step;

  acc_max(robotPt, sub_hilflat_dataPt, tab_acc_max, du);
  
  sub_hilflat_dataPt->v_1_rob_max = tab_acc_max[0];
  sub_hilflat_dataPt->w_1_rob_max = tab_acc_max[1];
  sub_hilflat_dataPt->gamma_1_min = tab_acc_max[2];   
  sub_hilflat_dataPt->theta_1_tot = tab_acc_max[3];

  return sub_hilflat_dataPt;
}

/*****************************************************************************/
/*compute an array of v_1_rob_max,w_1_rob_max, gamma_1_min*/
/* theta_max */
/*by equal steps*/
/*later the steps should depend on the dimention of the path*/
/*****************************************************************************/

static void acc_max(p3d_rob *robotPt, 
		    p3d_sub_hilflat_data * sub_hilflat_dataPt, 
		    double *tab_acc_max, double du)
{
  double velocity_rob[4];
  double v_1_rob, w_1_rob, v_1_rob_max = 0.0, w_1_rob_max = 0.;
  double tab_gamma[2*(MAX_DERIV+1)];
  double gamma_1, gamma_1_min=0;

  double v2 = sub_hilflat_dataPt->v;
  configPt q_init = sub_hilflat_dataPt->q_init;
  configPt q_end = sub_hilflat_dataPt->q_end;

  double u_start = sub_hilflat_dataPt->u_start;
  double u_end = sub_hilflat_dataPt->u_end;
  double u = u_start;
  double theta_rob_prev=0, theta_rob=0;
  double w_rob, w_rob_prev=0;
  double dy, z0, z1;
  double du2 = du*du;
  double theta_1_tot = 0;

  int init = 1;
  configPt q;
  int deriv_order = 3;
  
  while (u <= u_end){

    q = p3d_combination(robotPt, q_init, q_end, u, v2, deriv_order, tab_gamma);
    
    theta_rob = q[3]; 

    p3d_accelCoefficient(robotPt, sub_hilflat_dataPt, u, velocity_rob);
    /* angular velocity*/
    w_rob = fabs(velocity_rob[2]);
    /* linear acceleration */
    v_1_rob = fabs(velocity_rob[1]);
    /* angular acceleration */
    w_1_rob = fabs(velocity_rob[3]);

    v_1_rob_max = MAX(v_1_rob_max, v_1_rob);
    w_1_rob_max = MAX(w_1_rob_max, w_1_rob);
    
    if (fabs(u-u_start) > EPS6){
      /* estimate bounds on acceleration from successive positions */
      dy = diff_angle(theta_rob_prev, theta_rob);
      z0 = w_rob_prev;
      z1 = w_rob;

      w_1_rob_max = MAX(w_1_rob_max, fabs(6*dy + du*(2*z1+4*z0))/du2);
      w_1_rob_max = MAX(w_1_rob_max, fabs(-6*dy + du*(4*z1+2*z0))/du2);
    }

    gamma_1 = norme_gamma_1(robotPt, q_init, q_end, u, v2);
    if (init) {
      gamma_1_min = gamma_1;
      init = 0;
    }
    gamma_1_min = MIN(gamma_1_min, gamma_1);

    /* store position and orientation of robot */
    theta_rob_prev = theta_rob;
    w_rob_prev = w_rob;

    /* desallocate q */
    p3d_destroy_config(robotPt, q);

    theta_1_tot += du*fabs(w_1_rob);
    u += du;

  }
  tab_acc_max[0] = v_1_rob_max;
  tab_acc_max[1] = w_1_rob_max;
  tab_acc_max[2] = gamma_1_min;
  tab_acc_max[3] = theta_1_tot;
}



/**********************************************************************/
/* this function is taked from /trail/...*/
/*coeff[0] = v_rob */ 
/*coeff[1] = v_1_rob*/
/*coeff[2] = w_rob */
/*coeff[3] = w_1_rob */
/**********************************************************************/
static void p3d_accelCoefficient(p3d_rob *robotPt, 
				 p3d_sub_hilflat_data *sub_hilflat_dataPt, 
				 double u, double *coeff)
{
  FLAT_LOCAL_PATH_STR flatpath;
  hilflat_conv_sub_path_fpath( robotPt, sub_hilflat_dataPt, &flatpath);
  flatHilareAccelCoefficient(&flatpath, u, coeff);
}

/**********************************************************************/
/* this is the just the allcation of a hilflat localpath with some */
/* choice of "do we need to allocate a cusp_end sub path ?" */
/**********************************************************************/
static p3d_hilflat_data *p3d_alloc_spec_hilflat_localpath(p3d_rob *robotPt, 
							  configPt q_init, 
							  configPt q_cusp, 
							  configPt q_end, 
							  double u_start, 
							  double u_end,
							  int symmetric)
{
  p3d_hilflat_data *hilflat_data;

  int cas;
  double new_u_start, new_u_end, u_switch;
  hilflat_data = MY_ALLOC(p3d_hilflat_data, 1);

  hilflat_data->symmetric = symmetric;
/*first part to put u_start and u_end in right bounds*/
  if (q_end == NULL){
    new_u_start = MIN(u_start, PAR_END);
    new_u_end   = MIN(u_end, PAR_END);
  } else {
    new_u_start = MIN(u_start, 2 * PAR_END);
    new_u_end   = MIN(u_end, 2 * PAR_END);   
  }
  new_u_start   = MAX(new_u_start, PAR_INIT);
  new_u_end     = MAX(new_u_end, PAR_INIT);

  /*just to be sure that the end is after the start*/
  if (new_u_end < new_u_start) {
    u_switch = new_u_start;
    new_u_start = new_u_end;
    new_u_end = u_switch;
  }

/*find in witch case we are*/
/*the first digit is the part where is u_start and the second where is u_end*/
  if (new_u_end <= PAR_END)
    { cas = 11; }
  else {
    if (new_u_start >= PAR_END)
      { cas = 22; }
    else
      { cas = 12; }
  }
  switch (cas) {
  case 11 : 
    hilflat_data->init_cusp = 
      p3d_alloc_spec_hilflat_sub_localpath(robotPt, 
					   q_init, q_cusp, 
					   new_u_start, new_u_end, 
					   symmetric);
    hilflat_data->cusp_end = NULL;
    break;
  case 22 :
    hilflat_data->init_cusp = 
      p3d_alloc_spec_hilflat_sub_localpath(robotPt, 
					   q_cusp, q_end,
					   new_u_start - PAR_END, 
					   new_u_end - PAR_END, 
					   symmetric);
    hilflat_data->cusp_end = NULL;
    break;
  case 12 :
    hilflat_data->init_cusp = 
      p3d_alloc_spec_hilflat_sub_localpath(robotPt, 
					   q_init, q_cusp,
					   new_u_start, 
					   PAR_END, 
					   symmetric);
    hilflat_data->cusp_end = 
      p3d_alloc_spec_hilflat_sub_localpath(robotPt, 
					   q_cusp, q_end,
					   PAR_INIT, 
					   new_u_end - PAR_END, 
					   symmetric);
    break;
  }
  
  return hilflat_data;
}
/**********************************************************************/
/* allocation of local path of type hilflat */
/**********************************************************************/
p3d_localpath * p3d_alloc_hilflat_localpath (p3d_rob *robotPt, 
					     configPt q_init, 
					     configPt q_cusp, 
					     configPt q_end, 
					     double u_start, 
					     double u_end, 
					     int lp_id,
					     int symmetric)
{
  p3d_localpath * localpathPt;

  localpathPt = MY_ALLOC(p3d_localpath, 1);
  if (localpathPt == NULL)
    { return NULL; }

  /* allocation of the specific part */
  localpathPt->specific.hilflat_data = 
    p3d_alloc_spec_hilflat_localpath(robotPt, q_init, q_cusp, q_end, 
				     u_start, u_end, symmetric);

  if (localpathPt->specific.hilflat_data == NULL){
    /* allocation failed free everything and return NULL*/
    MY_FREE(localpathPt, p3d_localpath, 1);
    return NULL;
  }

  /* Initialization of the generic part */
  /* fields */
  localpathPt->type_lp = HILFLAT;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length = 
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_hilflat_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath = 
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, 
			double, double))(p3d_extract_hilflat);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param = 
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, 
			double, double))(p3d_extract_hilflat_by_param);
  /* destroy the localpath */
  localpathPt->destroy = 
    (void (*)(p3d_rob*, p3d_localpath*))(p3d_hilflat_destroy);
  /*copy the local path */
  localpathPt->copy = 
    (p3d_localpath* (*)(p3d_rob*, 
			p3d_localpath*))(p3d_copy_hilflat_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =   
    (configPt (*)(p3d_rob*, 
		  p3d_localpath*, double))(p3d_hilflat_config_at_distance); 
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =   
    (configPt (*)(p3d_rob*, p3d_localpath*, 
		  double))(p3d_hilflat_config_at_param); 
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The 
     function returns the half length of the interval */     
  localpathPt->stay_within_dist =   
    (double (*)(p3d_rob*, p3d_localpath*, 
		double, whichway, double*))(p3d_hilflat_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost = 
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_hilflat_cost);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  localpathPt->simplify = 
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_hilflat);
  /* write the local path in a file */
  localpathPt->write = 
    (int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_hilflat);

  localpathPt->length_lp = p3d_hilflat_dist(robotPt, localpathPt);
  localpathPt->range_param = u_end-u_start;
  localpathPt->ikSol = NULL;

  return localpathPt;
}
/***********************************************************************/
/* this function return valid but if the path as one time a theta  */
/* bigger than his bound or a gamma_1_min to small it could return */
/* false (only could because we test only by steps) */
/*********************************************************************/
int is_valid_hilflat(p3d_rob *robotPt, 
		     p3d_hilflat_data *hilflat_dataPt,
		     int symmetric)
{

  if (hilflat_face_to_face(robotPt, hilflat_dataPt->init_cusp)){
    return FALSE;
  }
  
  if ( !are_close (robotPt, hilflat_dataPt->init_cusp->q_init, 
		   hilflat_dataPt->init_cusp->q_end)){
    if (!is_v_valid(robotPt, hilflat_dataPt->init_cusp, symmetric)){
      return FALSE;
    }
    if (!is_gamma_1_min_valid(robotPt, hilflat_dataPt->init_cusp)){
      return FALSE;
    } 
  }
  else {
    if (v_cusp(robotPt, hilflat_dataPt->init_cusp->q_init, 
	       hilflat_dataPt->init_cusp->q_end) != -1){
      return FALSE;
    }
  }

  if (hilflat_dataPt->cusp_end != NULL){
    if ( !are_close (robotPt, hilflat_dataPt->cusp_end->q_init, 
		     hilflat_dataPt->cusp_end->q_end)){
      if (!is_v_valid(robotPt, hilflat_dataPt->cusp_end, symmetric)){
	return FALSE;
      }
      if (! is_gamma_1_min_valid(robotPt, hilflat_dataPt->cusp_end)){
	return FALSE;
      } 
    }
    else {
      if (v_cusp(robotPt, hilflat_dataPt->cusp_end->q_init, 
		 hilflat_dataPt->cusp_end->q_end) != -1){
	return FALSE;
      }
    }
  }
  return TRUE;
}
/**********************************************************************/
/* distance for the sub hilflat local method */
/*proportionnal of u_end - u_start*/
/**********************************************************************/
double p3d_sub_hilflat_distance(p3d_sub_hilflat_data *sub_hilflat_dataPt){
  double d ;/*the distance of the sub path*/
  double length;
  double u_start;
  double u_end;

  if (sub_hilflat_dataPt == NULL){
    return 0.;
  }
  else {
    length = sub_hilflat_dataPt->length;
    u_start = sub_hilflat_dataPt->u_start;
    u_end = sub_hilflat_dataPt->u_end;
    d = (u_end - u_start)/(PAR_END - PAR_INIT) * length;
  }
  return d;
}
/**********************************************************************/
/* distance for the hilflat local method */
/*this the sum of the distances of the 2 sub path*/
/**********************************************************************/
double p3d_hilflat_dist(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  /* cast the pointer to union p3d_hilflat_specific to a pointer
     to p3d_hilflat_data */
  p3d_hilflat_data *specificPt = localpathPt->specific.hilflat_data;
  p3d_sub_hilflat_data *init_cusp;
  p3d_sub_hilflat_data *cusp_end;
  double d, d_init_cusp, d_cusp_end;

  if (localpathPt->type_lp != HILFLAT){
    PrintError(("p3d_hilflat_dist: hilflat local local path expected\n"));
    return 0;
  }
  init_cusp = specificPt->init_cusp;
  cusp_end = specificPt->cusp_end;
  
  d_init_cusp = p3d_sub_hilflat_distance(init_cusp);
  d_cusp_end = p3d_sub_hilflat_distance(cusp_end);

  d = d_init_cusp + d_cusp_end;

  return(d);
}
/**********************************************************************/
/*destrucion of a sub_path*/
/* q_init_is_there is in case of q_end would already have been free */
/**********************************************************************/
void p3d_destroy_sub_hilflat_data(p3d_rob* robotPt,
				  p3d_sub_hilflat_data* sub_hilflat_dataPt,
				  int q_init_is_there)
{
  if (q_init_is_there){
    p3d_destroy_config(robotPt, sub_hilflat_dataPt->q_init);
  }
  p3d_destroy_config(robotPt, sub_hilflat_dataPt->q_end);
  MY_FREE(sub_hilflat_dataPt, p3d_sub_hilflat_data, 1);
}
/**********************************************************************/
 /* destroys a structure of type p3d_hilflat_data */
/**********************************************************************/
void p3d_destroy_hilflat_data(p3d_rob* robotPt, 
			      p3d_hilflat_data* hilflat_dataPt)
{
  p3d_destroy_sub_hilflat_data(robotPt, hilflat_dataPt->init_cusp, 1);
  if (hilflat_dataPt->cusp_end != NULL){
    p3d_destroy_sub_hilflat_data(robotPt, hilflat_dataPt->cusp_end, 0);
  }
  MY_FREE(hilflat_dataPt, p3d_hilflat_data, 1);
}
/**********************************************************************/
/* destroy a hilflat local path */
/**********************************************************************/

void p3d_hilflat_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt)
{
  if (localpathPt != NULL){

    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != HILFLAT){
      PrintError(("p3d_hilflat_destroy: hilflat local path expected\n"));
    }
    /* destroy the specific part */
    if (localpathPt->specific.hilflat_data != NULL){
      p3d_destroy_hilflat_data(robotPt, localpathPt->specific.hilflat_data);
    }
    localpathPt->next_lp = NULL;
    localpathPt->prev_lp = NULL;
    if (localpathPt->ikSol){
      p3d_destroy_specific_iksol(robotPt->cntrt_manager, localpathPt->ikSol);
      localpathPt->ikSol = NULL;
    }
    MY_FREE(localpathPt, p3d_localpath, 1);
  }
}
/**********************************************************************/
/* Compute the configuration situated at given distance on the sub local path.*/
/**/
/*  Input:  the robot, the distance u_lp on the lsub ocalpath  from the start*/
/*    and the sub localpath*/
/**/
/*  Output: the configuration*/
/**********************************************************************/
configPt p3d_sub_hilflat_config_at_distance(p3d_rob *robotPt, 
					    p3d_sub_hilflat_data *sub_hilflat_dataPt,
					    double d_lp)
{
  configPt q;
  double length = sub_hilflat_dataPt->length;
  double u_start = sub_hilflat_dataPt->u_start;
  double u_end = sub_hilflat_dataPt->u_end;
  double d_start,d_end, d, u, u_lp;

  d_start = u_start / (PAR_END - PAR_INIT) * length;
  d_end = u_end / (PAR_END - PAR_INIT) * length;
  d = d_lp + d_start;
  d = ((d > d_end) ? d_end : d);

  u = d / length * (PAR_END - PAR_INIT);  
  u_lp =  u - u_start;

  q = p3d_sub_hilflat_config_at_param(robotPt, sub_hilflat_dataPt, u_lp);
  
  return q;
}
/**********************************************************************/
/* Compute the configuration situated at given parameter on the sub local path.*/
/**/
/*  Input:  the robot, the distance u_lp on the lsub ocalpath  from the start*/
/*    and the sub localpath*/
/**/
/*  Output: the configuration*/
/**********************************************************************/
configPt 
p3d_sub_hilflat_config_at_param(p3d_rob *robotPt, 
				p3d_sub_hilflat_data *sub_hilflat_dataPt,
				double u_lp)
{
  configPt q;
  configPt q_init = sub_hilflat_dataPt->q_init;  
  configPt q_end = sub_hilflat_dataPt->q_end;
  double Tab_gamma[2*(MAX_DERIV+1)];
  double v = sub_hilflat_dataPt->v;
  double u_start = sub_hilflat_dataPt->u_start;
  double u_end = sub_hilflat_dataPt->u_end;
  double u = u_start + u_lp;
  int deriv_order = 3;
  
  u = MIN(u, u_end);
  u = MAX(u, u_start);

  q = p3d_combination(robotPt, q_init, q_end, u, v, 
		      deriv_order, Tab_gamma);

  return q;
}
/**********************************************************************/
/* Compute the configuration situated at given distance on the local path.*/
/**/
/*  Input:  the robot, the distance u_lp on the localpath  from the start*/
/*    and the localpath*/
/**/
/*  Output: the configuration*/
/**********************************************************************/
configPt p3d_hilflat_config_at_distance(p3d_rob *robotPt, 
					p3d_localpath *localpathPt,
					double d_lp)
{
  configPt q;
  p3d_sub_hilflat_data *init_cusp;
  p3d_sub_hilflat_data *cusp_end;  
  double length1;
  double lg_traj1;/*this is the length between u_start and u_end*/
  
  if (localpathPt == NULL)
    return NULL;
  if (localpathPt->type_lp != HILFLAT){
    PrintError(("p3d_hilflat_config_at_distance: local path must be hilflat\n"));
    return NULL;
  }
  init_cusp = localpathPt->specific.hilflat_data->init_cusp;
  if (init_cusp == NULL)
    return NULL;

  length1 = init_cusp->length;

  lg_traj1 = (init_cusp->u_end - init_cusp->u_start) / (PAR_END - PAR_INIT) * length1;

  if (d_lp <= lg_traj1){
    q = p3d_sub_hilflat_config_at_distance(robotPt, init_cusp, d_lp);
  }
  else {   
    cusp_end = localpathPt->specific.hilflat_data->cusp_end;  
    if (cusp_end == NULL){
      q = p3d_sub_hilflat_config_at_distance(robotPt, init_cusp, lg_traj1);
    }
    else {
      q = p3d_sub_hilflat_config_at_distance(robotPt, cusp_end, d_lp - lg_traj1);
    }
  }

  return q;
}
/**********************************************************************/
/* Compute the configuration situated at given parameter on the local path.*/
/**/
/*  Input:  the robot, the parameter u_lp  on the localpath  */
/*    and the localpath*/
/**/
/*  Output: the configuration*/
/**********************************************************************/
configPt p3d_hilflat_config_at_param(p3d_rob *robotPt, 
				     p3d_localpath *localpathPt,
				     double u_lp)
{
  configPt q;
  p3d_sub_hilflat_data *init_cusp;
  p3d_sub_hilflat_data *cusp_end;  
  double u_start1, u_end1;
  double u;
  
  if (localpathPt == NULL)
    return NULL;
  if (localpathPt->type_lp != HILFLAT){
    PrintError(("p3d_hilflat_config_at_distance: local path must be hilflat\n"));
    return NULL;
  }

  init_cusp = localpathPt->specific.hilflat_data->init_cusp;
  if (init_cusp == NULL)
    return NULL;
  u_start1 = init_cusp->u_start;
  u_end1 = init_cusp->u_end;
  
  u = u_start1 + u_lp;
  if ( u <= u_end1){
    q = p3d_sub_hilflat_config_at_param(robotPt, init_cusp, u_lp);
  }
  else {
    cusp_end = localpathPt->specific.hilflat_data->cusp_end;  
    if (cusp_end == NULL){
      q = p3d_sub_hilflat_config_at_param(robotPt, init_cusp, 
					  u_end1 - u_start1);
    }
    else {
      q = p3d_sub_hilflat_config_at_param(robotPt, cusp_end, u - u_end1);
    }
  }
  return q;
}

/**********************************************************************/
/*this give the length v_cusp on the curve of gamma_2 where to put the cusp*/
/*return -1 if we don't have to put a cusp*/
/*else return the v where to put the cusp*/
/**********************************************************************/
static double v_cusp(p3d_rob *robotPt, configPt q1, configPt q2){
  TR_FLAT_CONFIG_STR qf1, qf2;
  conv_conf_fconf(robotPt, q1, &qf1);
  conv_conf_fconf(robotPt, q2, &qf2);
  return v_fcusp(robotPt, &qf1, &qf2);
}
/**********************************************************************/
/*this give the length v_cusp on the curve of gamma_2 where to put the cusp*/
/*return -1 if we don't have to put a cusp*/
/*else return the v where to put the cusp*/
/**********************************************************************/
static double v_fcusp(p3d_rob *robotPt,
		      TR_FLAT_CONFIG_STR *q1, 
		      TR_FLAT_CONFIG_STR *q2)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double diam_rob = robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;
  double v2 = flatV2(q1, q2);
  double rho2 = fabs(flatV1(q1, q2));
  double theta2 = q2->tau;
  double v_cond2=0, v_cond3=0;
  TR_FLAT_CONFIG_STR *q1_barre = MY_ALLOC(TR_FLAT_CONFIG_STR,1);
  /*the configuration of the projection of q2 on Gamma(q1,s)*/
  double kappa1_barre,theta1_barre;
  double Tab_gamma[2*(MAX_DERIV + 1)];
  double vv_cusp;

  flatGamma(q1, v2, 1, Tab_gamma);
  flat_conv_curve_fconf (Tab_gamma, q1_barre, 0.);
 
  kappa1_barre = q1_barre->kappa;
  theta1_barre = q1_barre->tau;

  MY_FREE(q1_barre, TR_FLAT_CONFIG_STR,1);

  v_cond2 = diam_rob*fabs(theta2 - theta1_barre);
  v_cond3 = sqrt(fabs(rho2)*diam_rob);

  if (fabs(v2) >= v_cond2 && fabs(v2) >= v_cond3){
    return (-1);
  }
  else {
    vv_cusp = MAX(v_cond2,v_cond3);
    return vv_cusp;
  }
}

static configPt middle_config(p3d_rob *robotPt, configPt q_1, configPt q_2)
{
  configPt q_middle = p3d_alloc_config(robotPt);
  int i, j, k, njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for (i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      q_middle[k] = p3d_jnt_calc_dof_value(jntPt, j, q_1, q_2, 0.5);
    }
  }
  return q_middle;
}


/**********************************************************************/
/*compute the cusp if necessary*/
/* old version where the cusp is on the circle of q2 we just chose  */
/* in  which way */
/**********************************************************************/
/* int cusp_compute(p3d_rob *robotPt, configPt q1, configPt q2,  */
/* 		 configPt *q_cusp){ */
/*   double v = v_cusp(robotPt, q1, q2); */
/*   double Tab_gamma[2*(MAX_DERIV + 1)]; */
/*   configPt q_cusp1, q_cusp2; */
/*   double d_cone1, d_cone2; */
  
/*   if (v == -1){ */
/*     return 0; */
/*   } */
/*   else { */
/*     q_cusp1 = p3d_Gamma(robotPt, q2, v, Tab_gamma); */
/*     q_cusp2 = p3d_Gamma(robotPt, q2, -v, Tab_gamma); */

/*     d_cone1 = dist_cone(robotPt, q1, q_cusp1); */
/*     d_cone2 = dist_cone(robotPt, q1, q_cusp2); */
/*     if (d_cone1 <= d_cone2){ */
/*       *q_cusp = p3d_copy_config(robotPt, q_cusp1);  */
/*       p3d_destroy_config(robotPt, q_cusp1); */
/*       p3d_destroy_config(robotPt, q_cusp2); */
/*     } */
/*     else { */
/*       *q_cusp = p3d_copy_config(robotPt, q_cusp2);  */
/*       p3d_destroy_config(robotPt, q_cusp1); */
/*       p3d_destroy_config(robotPt, q_cusp2); */
/*     } */
/*     return 1; */
/*   } */
/* } */
/**********************************************************************/
/*compute the cusp if necessary*/
/**********************************************************************/
static int cusp_compute(p3d_rob *robotPt, configPt q1, configPt q2, 
			configPt *q_cusp)
{
  double v = v_cusp(robotPt, q1, q2);
  double Tab_gamma[2*(MAX_DERIV + 1)];
  configPt q_cusp_init1, q_cusp_init2, q_cusp_end1, q_cusp_end2;
  configPt q_cusp1 = NULL;
  configPt q_cusp2 = NULL;

  double d_cone1, d_cone2;
  double d_init1_end1, d_init1_end2;

  if (v == -1){
    return 0;
  }
  else {
    /* first step : */
    /* we find 4 point on the circle of q1 and q2 at the distance of v in both direction*/
    q_cusp_init1 = p3d_Gamma(robotPt, q1, v, Tab_gamma);
    q_cusp_init2 = p3d_Gamma(robotPt, q1, -v, Tab_gamma);
    q_cusp_end1 = p3d_Gamma(robotPt, q2, v, Tab_gamma);
    q_cusp_end2 = p3d_Gamma(robotPt, q2, -v, Tab_gamma);
    
    /* second step : */
    /* we find the right combination of the 4 cusp because we don't want to have a point in the middle of q1 and q2*/
    d_init1_end1 = dist_conf(robotPt, q_cusp_init1, q_cusp_end1);
    d_init1_end2 = dist_conf(robotPt, q_cusp_init1, q_cusp_end2);

    if (d_init1_end1 <= d_init1_end2){
      /*here we compute the mean of the cusp*/
      q_cusp1 = middle_config(robotPt, q_cusp_init1, q_cusp_end1);
      q_cusp2 = middle_config(robotPt, q_cusp_init2, q_cusp_end2);
    }
    else {
      /*here we compute the mean of the cusp*/
      q_cusp1 = middle_config(robotPt, q_cusp_init1, q_cusp_end2);
      q_cusp2 = middle_config(robotPt, q_cusp_init2, q_cusp_end1);
    }

    /* now we just have two cusp point and we chose the closer of both cone */
    /* if we have chance we have a point on both cone */
    d_cone1 = (dist_cone(robotPt, q1, q_cusp1) + 
	       dist_cone(robotPt, q2, q_cusp1))/2;
    d_cone2 = (dist_cone(robotPt, q1, q_cusp2) + 
	       dist_cone(robotPt, q2, q_cusp2))/2;
    
    if (d_cone1 <= d_cone2){
      *q_cusp = q_cusp1; 
      p3d_destroy_config(robotPt, q_cusp2);
    }
    else {
      *q_cusp = q_cusp2; 
      p3d_destroy_config(robotPt, q_cusp1);
    }
    
    p3d_destroy_config(robotPt,q_cusp_init1);
    p3d_destroy_config(robotPt,q_cusp_init2);
    p3d_destroy_config(robotPt,q_cusp_end1);
    p3d_destroy_config(robotPt,q_cusp_end2);
    return 1;
  }
}

/**********************************************************************/
/*as cusp_compute but with the right order to keep symetry of the localpath*/
/**********************************************************************/
static int cusp_compute_order(p3d_rob *robotPt, configPt q1, configPt q2, 
			      configPt *q_cusp, int symmetric){
  int res;
  configPt q_min;
  configPt q_max;
  whichway dir;
  dir = order_conf(robotPt, q1, q2, &q_min, &q_max, symmetric);
  res = cusp_compute(robotPt, q_min, q_max, q_cusp);
 
  return res;
}

/*
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input. 
 *
 *  Input: speed of the previous joint (prev_data),
 *         the robotPt (robotPt),
 *         the maximal distances (distance_base, distance_hilflat),
 *         the final configuration (q_max_param),
 *         the current maximal parameter that could be reach (reach_param)
 *	   the parameter (u_lp)
 *         the sub-structure of localpath (sub_hilflat_dataPt)
 *
 * Output: speed of the joints (base_data, hilflat_data),
 *         the distances that the joint couldn't cross 
 *                       (distance_base, distance_hilflat),
 *         the actual maximal parameter that could be reach (reach_param)
 */
static void p3d_jnt_hilflat_stay_within_dist(
			     p3d_stay_within_dist_data * prev_data,
			     p3d_rob * robotPt,
			     p3d_stay_within_dist_data * base_data,
			     double * distance_base,
			     configPt q_init,
			     configPt q_max_param, double *reach_param,
			     double u_lp, p3d_sub_hilflat_data 
			     *sub_hilflat_dataPt)
{
  double d_rob, diam_rob;
  p3d_jnt * jnt_basePt, *jnt_xPt, *jnt_yPt;
  int i_dof_x, i_dof_y, x_coord, y_coord;
  
  /* upper bounds on acceleration of all points of robot and hilflat
     along local path. These bounds are computed when local path is 
     created */
  double v_1_rob_max = sub_hilflat_dataPt->v_1_rob_max;
  double w_1_rob_max = sub_hilflat_dataPt->w_1_rob_max;
  double v_rob, w_rob;
  double v_all_rob;
  double delta_rob;
  
  double min_range_rob;
  double velocity_rob[4];
  double u = u_lp + sub_hilflat_dataPt->u_start;
  double w_max_rob=0;
  p3d_point p_min, p_max;
  double dist_base, dist_hilflat;

  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  jnt_basePt    = robotPt->joints[hilflat_params->numjnt[JNT_BASE]];
  x_coord = hilflat_params->numdof[HILFLAT_DOF_X];
  y_coord = hilflat_params->numdof[HILFLAT_DOF_Y];
  jnt_xPt       = p3d_robot_dof_to_jnt(robotPt, x_coord, &i_dof_x);
  jnt_yPt       = p3d_robot_dof_to_jnt(robotPt, y_coord, &i_dof_y);
  d_rob = *distance_base;
  diam_rob = jnt_basePt->dist;

  p3d_jnt_get_point(jnt_basePt,    &(base_data->p));
  
  /* distance between the reference point of the previous body
     and the point the current joint is attached to
     (this is only an approximation) */
  if (prev_data->wmax < EPS6) { /* We don't need to compute this distance */
    dist_base = 0.;
    dist_hilflat = 0.;
  } else {
    p_min.x = base_data->p.x + 
      q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
      q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
    p_min.y = base_data->p.y +
      q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
      q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
    p_min.z = base_data->p.z +
      q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
      q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
    
    p_max.x = base_data->p.x + 
      q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
      q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
    p_max.y = base_data->p.y +
      q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
      q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
    p_max.z = base_data->p.z + 
      q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[2] +
      q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[2];

    dist_base = MAX(p3d_point_dist(prev_data->p, p_min),
		    p3d_point_dist(prev_data->p, p_max));
    
  }

  /* compute maximal velocity of all points of plateform and hilflat */
  v_1_rob_max = v_1_rob_max + diam_rob * (w_1_rob_max + prev_data->wmax)
    + dist_base * prev_data->wmax + prev_data->vmax;
  /* compute linear and angular velocities of robot at given 
     parameter */
  p3d_accelCoefficient(jnt_basePt->rob, sub_hilflat_dataPt, u, velocity_rob);
  w_rob = fabs(velocity_rob[2]);
  v_rob = fabs(velocity_rob[0]);

  /* bounds on velocity of all points of robot and hilflat */
  v_all_rob =  v_rob + w_rob * diam_rob;

  /* We compute, respectively for the platform and for the hilflat, a 
     range that ensures that not point of the body move by more than
     d_rob, respectively d_rem 

     these ranges satisfy the following equations in du:

                        1        2
          v    du     + - a    du  = d
           max          2  max

    where v    =  v_all_rob
           max	  

          a    = v_1_rob_max 
           max
  */

  delta_rob = v_all_rob*v_all_rob + 2 * d_rob * v_1_rob_max;

  if (fabs(v_1_rob_max) > NEAR_ZERO)
    { min_range_rob = (sqrt(delta_rob) - v_all_rob)/ v_1_rob_max; }
  else {
    if (fabs(v_all_rob) > NEAR_ZERO) 
      { min_range_rob = d_rob / v_all_rob; }
    else
      { min_range_rob = 0.; }
  }

  /* set distances to know that robot or hilflat moved up to 
     its maximum */
  if (min_range_rob <= (*reach_param)) {
    /* robot can reach end of local path by moving less than its
       specified distance */
    *reach_param = min_range_rob;
  }
  
  (*distance_base) -= v_all_rob * (*reach_param) + 
    .5*v_1_rob_max *SQR(*reach_param);

  /* maximal angular velocities of robot over interval */
  w_max_rob = w_rob + (*reach_param)*w_1_rob_max;

  /* bounds on linear and angular velocities of the plateform */
  base_data->vmax = v_rob + (*reach_param)*v_1_rob_max;
  base_data->wmax = w_rob + (*reach_param)*w_1_rob_max;
}


static double p3d_sub_hilflat_stay_within_dist(p3d_rob* robotPt,
					       p3d_sub_hilflat_data 
					       *sub_hilflat_dataPt,
					       double u_lp, whichway dir,
					       double *distances, int valid)
{
  int base_joint = 0;
  
  double the_min_range, max_param, min_param;
  double u_start = sub_hilflat_dataPt->u_start;
  double u_end = sub_hilflat_dataPt->u_end;
  double u = u_lp + u_start;
  int i, j, k, njnt = robotPt->njoints;
  p3d_jnt *prev_jntPt=NULL, *cur_jntPt=NULL;
  configPt q_max_param, q_param;
  p3d_stay_within_dist_data * stay_within_dist_data;

  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double diam_rob = 
    robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;
  base_joint = hilflat_params->numjnt[JNT_BASE];

  the_min_range = dir*fabs(diam_rob/sub_hilflat_dataPt->length)/10.;

  /* range_max is the maximal range possible whitout reaching bounds
     of local path. Notive that ranges are always positive */
  if (dir == FORWARD) {
    min_param = max_param = u_end - u;
    q_max_param = sub_hilflat_dataPt->q_end;
  } else {
    min_param = max_param = u - u_start;
    q_max_param = sub_hilflat_dataPt->q_init;    
  }
  /* if local path is not valid, return fake range (just for display) */
  if (the_min_range > max_param) 
    { the_min_range = max_param; }

  if (!valid){
    return the_min_range;
  }
  /* Get the current config to have the modifications of the constraints */
  q_param = p3d_get_robot_config(robotPt);

  /* store the data to compute the maximal velocities at the 
     joint for each body of the robot */
  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
  p3d_init_stay_within_dist_data(stay_within_dist_data);

  /* we have now to take into account the joints */
  k = 0; /* index of other joints */
  for (i=0; i <= njnt; i++) {
    cur_jntPt = robotPt->joints[i];
    prev_jntPt = cur_jntPt->prev_jnt;

    if (i==base_joint) { 
      /* We could compute the stay_within_dist for the hilflat method */

      /* j = index of the joint to which the current joint is attached */
      j = -2;
      do {
	if (prev_jntPt == NULL) 
	  { j = -1; }
	else {
	  if ((prev_jntPt->index_dof <= hilflat_params->numdof[HILFLAT_DOF_X]) &&
	      (prev_jntPt->index_dof <= hilflat_params->numdof[HILFLAT_DOF_Y])) {
	    if (prev_jntPt->prev_jnt == NULL)
	      { j = -1; }
	    else
	      { j = prev_jntPt->prev_jnt->num; }
	  } else 
	    { prev_jntPt = prev_jntPt->prev_jnt; }
	}
      } while(j==-2);
      p3d_jnt_hilflat_stay_within_dist(&(stay_within_dist_data[j+1]), robotPt,
			       &(stay_within_dist_data[base_joint+1]),
			       &(distances[base_joint]),
			       q_param, q_max_param, &min_param,
			       u_lp, sub_hilflat_dataPt);
      /* All the joints which compose the hilflat_joint have the same
	 stay_within_dist (this majoration is too strong, but normaly,
	 we couldn't have other link than base_joint, so we don't care) */
      while((cur_jntPt!=prev_jntPt) && (cur_jntPt->prev_jnt!=NULL)) {
	cur_jntPt = cur_jntPt->prev_jnt;
	stay_within_dist_data[cur_jntPt->num+1] =
	  stay_within_dist_data[base_joint+1];
      }      
    } else if ((k < hilflat_params->nb_other_jnt) && 
	       (i == hilflat_params->other_jnt[k])) {
      k ++;
    
      /* j = index of the joint to which the current joint is attached */
      if (prev_jntPt==NULL) 
	{ j = -1; } /* environment */
      else
	{ j = prev_jntPt->num; }
    
      p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
			       &(stay_within_dist_data[i+1]), &(distances[i]), 
			       q_param, q_max_param, max_param, &min_param);
      /* Rem: stay_within_dist_data[0] is bound to the environment */
    }
  }
  
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
  p3d_destroy_config(robotPt, q_param);

  return min_param;
}


/**********************************************************************/
/*   p3d_hilflat_stay_within_dist */
/*   */
/*  *  Input:  the robot, */
/*  *          the local path, */
/*  *          the parameter along the curve, */
/*  *          the maximal distances moved by all the points of the */
/*  *          bodies of the robot  */
/*  * */
/*  *  Output: half length of the interval of parameter the robot can */
/*  *          describe without moving by more than the input distance */

/*  * */
/*  *  Description: */
/*  *          From a configuration on a local path, this function */
/*  *          computes an interval of parameter on the local path on */
/*  *          which all the points of the robot move by less than the */
/*  *          distance given as input.  The interval is centered on the */
/*  *          configuration given as input. The function returns the */
/*  *          half length of the interval  */
/*  * */
/*  *  Note: */
/*  *          Before this function, we need a p3d_set_and_update_config */
/*  *          for the configuration at u_lp. This allow to take into */
/*  *          account a part of the constraints. */
/*  * */
/**********************************************************************/
/*version who find the distance with the local speed, 03/08/2K*/
double p3d_hilflat_stay_within_dist(p3d_rob* robotPt,
				    p3d_localpath* localpathPt,
				    double u_lp, whichway dir,
				    double *distances)
{
  p3d_sub_hilflat_data *init_cusp = localpathPt->specific.hilflat_data->init_cusp;
  p3d_sub_hilflat_data *cusp_end = localpathPt->specific.hilflat_data->cusp_end;
  double u = u_lp + init_cusp->u_start;
  double du;
  double range_param_1 = init_cusp->u_end - init_cusp->u_start;
  int valid = localpathPt->valid;

  if (u >= PAR_END){
    if (cusp_end != NULL){
      du = p3d_sub_hilflat_stay_within_dist(robotPt, cusp_end,
					    u_lp - range_param_1, dir,
					    distances, valid);
    }
    else {
      du = p3d_sub_hilflat_stay_within_dist(robotPt, init_cusp,
					    range_param_1, dir,
					    distances, valid);     
    }
  }
  else {
    du = p3d_sub_hilflat_stay_within_dist(robotPt, init_cusp,
					   u_lp , dir,
					   distances, valid);
  }
  return du;
}
/**********************************************************************/
/*  Copy one local path.*/
   /*  Input:  the robot, the local path.*/
   /*  Output: the copied local path*/
/**********************************************************************/
p3d_localpath *p3d_copy_hilflat_localpath(p3d_rob* robotPt, 
					  p3d_localpath* localpathPt)
{
  p3d_localpath *hilflat_localpathPt;
  p3d_sub_hilflat_data *init_cusp = localpathPt->specific.hilflat_data->init_cusp;
  p3d_sub_hilflat_data *cusp_end = localpathPt->specific.hilflat_data->cusp_end;
  
  configPt q_init = p3d_copy_config(robotPt, init_cusp->q_init); 

  configPt q_cusp;
  configPt q_end;
  double u_start ;
  double u_end ;

  int lp_id = localpathPt->lp_id;
  int valid = localpathPt->valid;
  int symmetric = localpathPt->specific.hilflat_data->symmetric;

  u_start = init_cusp->u_start;
  q_cusp = p3d_copy_config(robotPt, init_cusp->q_end);

  if (cusp_end == NULL){
    q_end = NULL;
    u_end = init_cusp->u_end;
  }
  else{
    q_end = p3d_copy_config(robotPt, cusp_end->q_end);
    u_end = PAR_END + cusp_end->u_end;
  }

  hilflat_localpathPt = p3d_alloc_hilflat_localpath(robotPt, q_init, 
						    q_cusp, q_end, 
						    u_start, u_end,
						    lp_id, symmetric);
  hilflat_localpathPt->valid = valid;
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(hilflat_localpathPt->ikSol));

  return hilflat_localpathPt;
}
/**********************************************************************/
/*  Extract from a sub hilflat local path the sub local path starting*/
     /*  at length l1 and ending at length l2.*/
     /*  The length of the extracted local path is computed*/
 /*  If l2 > length local path, return end of local path*/
/**********************************************************************/
void p3d_extract_sub_hilflat(p3d_rob *robotPt, 
			     p3d_sub_hilflat_data *sub_hilflat_dataPt,
			     double d_lp1, double d_lp2, double *Tab_new_u)
{
  double length = sub_hilflat_dataPt->length;
  double u_start = sub_hilflat_dataPt->u_start;
  double u_end = sub_hilflat_dataPt->u_end;
  double d_start, d_end, d1, d2;
  double new_u_start, new_u_end;

  d_start = u_start / (PAR_END - PAR_INIT) * length;
  d_end = u_end / (PAR_END - PAR_INIT) * length;

  d1 = d_lp1 + d_start;
  d2 = d_lp2 + d_start;
  d1 = ((d1 > d_end) ? d_end : d1);
  d2 = ((d2 > d_end) ? d_end : d2);
  new_u_start = d1 / length * (PAR_END - PAR_INIT);  
  new_u_end = d2 / length * (PAR_END - PAR_INIT);  

  Tab_new_u[0] = new_u_start;
  Tab_new_u[1] = new_u_end;
}
/**********************************************************************/
/*  Extract from a hilflat local path the sub local path starting*/
     /*  at length l1 and ending at length l2.*/
     /*  The length of the extracted local path is computed*/
 /*  If l2 > length local path, return end of local path*/
/**********************************************************************/
p3d_localpath *p3d_extract_hilflat(p3d_rob *robotPt, 
				   p3d_localpath *localpathPt,
				   double d_lp1, double d_lp2)
{
  p3d_localpath * sub_localpathPt;
  p3d_sub_hilflat_data *init_cusp;
  p3d_sub_hilflat_data *cusp_end;
  int lp_id = localpathPt->lp_id;
  int valid = localpathPt->valid;
  configPt q_init, q_cusp, q_end;
  int cas ;
  double d_lp_switch;
  double length, length1;
  double lg_traj1;/*this is the length between u_start and u_end*/
  
  double new_u[2], new_u_start, new_u_end;
  int symmetric = localpathPt->specific.hilflat_data->symmetric;
  
  if (localpathPt == NULL)
    return NULL;
  if (localpathPt->type_lp != HILFLAT){
    PrintError(("p3d_hilflat_dist: hilflat local local path expected\n"));
  }
  new_u_start = new_u_end = 0.;
  q_init = q_cusp = q_end = NULL;

  init_cusp = localpathPt->specific.hilflat_data->init_cusp;
  cusp_end = localpathPt->specific.hilflat_data->cusp_end;

  if (init_cusp == NULL)
    return NULL;
  
  if (d_lp1 > d_lp2){
    d_lp_switch = d_lp1;
    d_lp1 = d_lp2;
    d_lp2 = d_lp_switch;
  }
  
  length = localpathPt->length_lp;
  
  d_lp1 = ((d_lp1 > length) ? length : d_lp1);
  d_lp2 = ((d_lp2 > length) ? length : d_lp2);

  length1 = init_cusp->length;
  lg_traj1 = (init_cusp->u_end - init_cusp->u_start) / 
    (PAR_END - PAR_INIT) * length1;
 
  if (d_lp2 <= lg_traj1){
    cas = 11;
  }
  else {
    if (d_lp1 >= lg_traj1){
      cas = 22;
    }
    else {
      cas = 12;
    }
  }
  
  switch (cas) {
    case 11 : 
      p3d_extract_sub_hilflat(robotPt, init_cusp, d_lp1, d_lp2, new_u);
      new_u_start = new_u[0];
      new_u_end = new_u[1];
      q_init = p3d_copy_config(robotPt, init_cusp->q_init);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_init);
      }
      q_cusp = p3d_copy_config(robotPt, init_cusp->q_end);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_cusp, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_cusp);
      }
      q_end = NULL;
      break;
    case 22 :
      p3d_extract_sub_hilflat(robotPt, cusp_end, d_lp1 - lg_traj1, 
			      d_lp2 - lg_traj1, new_u);
      new_u_start = new_u[0];
      new_u_end = new_u[1];
      q_init = p3d_copy_config(robotPt, cusp_end->q_init);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_init);
      }
      q_cusp = p3d_copy_config(robotPt, cusp_end->q_end);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_cusp, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_cusp);
      }
      q_end = NULL;
      break;    
    case 12 :
      p3d_extract_sub_hilflat(robotPt, init_cusp, d_lp1, lg_traj1, new_u);
      new_u_start = new_u[0];
      p3d_extract_sub_hilflat(robotPt, cusp_end, 0., d_lp2 - lg_traj1, new_u);
      new_u_end = new_u[1] + PAR_END;
      q_init = p3d_copy_config(robotPt, init_cusp->q_init);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_init);
      }
      q_cusp = p3d_copy_config(robotPt, init_cusp->q_end);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_cusp, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_cusp);
      }
      q_end = p3d_copy_config(robotPt, cusp_end->q_end);
      if (robotPt->cntrt_manager->cntrts != NULL) {
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_end, NULL, 0, localpathPt->ikSol);
	p3d_get_robot_config_into(robotPt, &q_end);
      }
      break;    
  }
    
  sub_localpathPt = p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp, 
						q_end, new_u_start, 
						new_u_end, 
						lp_id, symmetric);
  sub_localpathPt->valid = valid;
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
  return sub_localpathPt;
}

/*
 *  Extract from a hilflat local path the sub local path starting
 *  at parameter u1 and ending at parameter u2.
 *  The length of the extracted local path is computed
 *
 *  If u2 > range_param local path, return end of local path
 */

p3d_localpath *p3d_extract_hilflat_by_param(p3d_rob *robotPt,
					    p3d_localpath *localpathPt,
					    double u1, double u2)
{
  double length = localpathPt->length_lp;
  double range_param = localpathPt->range_param;
  double l1, l2;

  l1 = length*u1/range_param;
  l2 = length*u2/range_param;

  return p3d_extract_hilflat(robotPt, localpathPt, l1, l2);
  
}


/**********************************************************************/
/* Cost of a local path*/
/* Input:  the local path*/
 /* Output: the cost  */
/**********************************************************************/
double p3d_hilflat_cost(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  double res;
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);
  double diam_robot = 
    robotPt->joints[hilflat_params->numjnt[JNT_BASE]]->dist;
 
  /*here the "* kappa_max / 2.5" if for not having a length */  
  /*the 2.5 is because when I put this the optimisze without*/
  /*the "* kappa_max / 2.5" is great and the current value of kappa_max is around 2.5*/
  res = localpathPt->length_lp/diam_robot + 
    localpathPt->specific.hilflat_data->init_cusp->theta_1_tot;
  
  if (localpathPt->specific.hilflat_data->cusp_end != NULL){
    res += localpathPt->specific.hilflat_data->cusp_end->theta_1_tot;
  }
  return res;
}
/**********************************************************************/
 /*  does nothing */
/**********************************************************************/
p3d_localpath *p3d_simplify_hilflat (p3d_rob *robotPt, 
				     p3d_localpath *localpathPt,
				     int *need_colcheck)
{
  return localpathPt;
}

/*
 *  p3d_write_hilflat --
 *
 *  write a localpath of type hilflat in a file 
 *
 *  ARGS IN  : a file descriptor, 
 *             a robot,
 *             a localpath
 *
 *  ARGS OUT : TRUE if success,
 *             FALSE if input local path is not a hilflat one.
 */

int p3d_write_hilflat(FILE *file, p3d_rob* robotPt, 
		      p3d_localpath* localpathPt)
{
  p3d_hilflat_data *hilflat_dataPt = NULL;
  p3d_sub_hilflat_data *init_cusp = NULL, *cusp_end=NULL;
  int symmetric = localpathPt->specific.hilflat_data->symmetric;

  if (localpathPt->type_lp != HILFLAT) {
    return FALSE;
  }

  if (symmetric) {
    fprintf(file, "\n\np3d_add_localpath HILFLAT\n");
  }
  else {
    fprintf(file, "\n\np3d_add_localpath NON_SYMMETRIC_HILFLAT\n");
  }    
	  
  hilflat_dataPt = (pp3d_hilflat_data)localpathPt->specific.hilflat_data;
  
  /* write first segment */
  init_cusp = hilflat_dataPt->init_cusp;
  cusp_end = hilflat_dataPt->cusp_end;

  
  fprintf(file, "conf_init");
  fprint_config_one_line(file, robotPt, init_cusp->q_init);
  fprintf(file, "\n");

  fprintf(file, "conf_cusp ");
  fprint_config_one_line(file, robotPt, init_cusp->q_end);
  fprintf(file, "\n");

  if (cusp_end != NULL) {
    fprintf(file, "conf_end ");
    fprint_config_one_line(file, robotPt, cusp_end->q_end);
    fprintf(file, "\n");
  }

  fprintf(file, "u_start\t%f\n", init_cusp->u_start);

  if (cusp_end != NULL) {
    fprintf(file, "u_end\t%f\n", cusp_end->u_end);
  }
  else {
    fprintf(file, "u_end\t%f\n", init_cusp->u_end);
  }    
  fprintf(file, "\n");

  fprintf(file, "\np3d_end_local_path\n");
  
  return TRUE;
}



/**********************************************************************/
/*compute the norme of the first derivation of the combination*/
/*cf trail/flat/src/general_hilflat.c */
/**********************************************************************/
static double norme_gamma (p3d_rob *robotPt, configPt q1, configPt q2, 
			   double u, double v2, int order){
  double Tab_gamma[2*(MAX_DERIV+1)];
  double res;
  configPt q;
  int deriv_order = 3;

  q = p3d_combination (robotPt, q1, q2, u, v2, 
		       deriv_order, Tab_gamma);
  p3d_destroy_config(robotPt, q);

  res = sqrt(pow(Tab_gamma[2*order],2.0)+pow(Tab_gamma[2*order+1],2.0));

  return res;
}

static double norme_gamma_1 (p3d_rob *robotPt, configPt q1, 
			     configPt q2, double u, 
			     double v2)
{
  return (norme_gamma(robotPt, q1, q2, u, v2, 1));
}

/**********************************************************************/
/**/
/*  Debugging functions*/
/**********************************************************************/
/**********************************************************************/

/* 
 *  lm_destroy_hilflat_params --
 *
 *  destroy data-structure specific to hilflat local method parameters
 */

void lm_destroy_hilflat_params(p3d_rob *robotPt, void *local_method_params)
{
  philflat_str hilflatPt = (philflat_str)local_method_params;
  
  MY_FREE(hilflatPt->other_jnt, int, hilflatPt->nb_other_jnt);
  MY_FREE(hilflatPt, hilflat_str, 1);
}

/*
 *  Local planner for a robot with a hilflat 
 */

p3d_localpath *p3d_hilflat_localplanner(p3d_rob *robotPt, double *qi, 
					double *qf, int* ikSol)
{
  p3d_localpath *localpathPt=NULL;
  configPt q_init, q_cusp, q_end;
  double u_start = PAR_INIT;
  double u_end = PAR_END;
  double v2;
  int cusp, i, j, k;
  double hilflat_var[NB_COORD_HILFLAT];
  p3d_jnt * jntPt;
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);

  if (hilflat_params == NULL){
    PrintInfo(("  Local method flat Hilare not initialized\n"));
    return NULL;
  }

  /* freeze dof between the hilflat kinematic chain */
  for(i=0; i<NB_COORD_HILFLAT; i++)
    { hilflat_var[i] = qf[hilflat_params->numdof[i]]; }
  j = 0;
  for(i=0; i<=robotPt->njoints; i++) {
    if ((j < hilflat_params->nb_other_jnt) &&
	(i == hilflat_params->other_jnt[j]))
      { j++; }
    else { /* this joint must be frozen */
      jntPt = robotPt->joints[i];
      for(k=0; k<jntPt->dof_equiv_nbr; k++)
	{ qf[k+jntPt->index_dof]= qi[k+jntPt->index_dof]; }
    }
  }
  for(i=0; i<NB_COORD_HILFLAT; i++)
    { qf[hilflat_params->numdof[i]] = hilflat_var[i]; }

  q_init = p3d_copy_config(robotPt, qi);
  q_cusp = p3d_copy_config(robotPt, qf);

  q_end = NULL;
  v2 = V2_order(robotPt, q_init, q_cusp, TRUE);

  if (v2 != 0){
    /* at first we try the direct way */
    localpathPt = p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp,
					      q_end,
					      u_start, u_end, 0, TRUE);
  
    localpathPt->valid = is_valid_hilflat(robotPt, 
					  localpathPt->specific.hilflat_data,
					  TRUE);
    if (localpathPt->valid == FALSE){
      p3d_hilflat_destroy(robotPt, localpathPt);
      localpathPt = NULL;
    }
  }
    
  if (localpathPt == NULL){
    
    /* second time we try to put a cusp FORWARD*/
    p3d_hilflat_destroy(robotPt, localpathPt);
    q_init = p3d_copy_config(robotPt, qi);
    q_end = p3d_copy_config(robotPt, qf);
    q_cusp = NULL;

    /*introduction of a cusp if necessary*/
    cusp = cusp_compute_order(robotPt, q_init,q_end,&q_cusp, TRUE);
    
    if (cusp){
      u_end = 2. * PAR_END;
    }
    else {
      p3d_destroy_config(robotPt, q_end);
      q_end = NULL;
      p3d_destroy_config(robotPt, q_cusp);
      q_cusp = p3d_copy_config(robotPt, qf);
    }    

    localpathPt = p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp,
					      q_end, u_start, u_end, 0,
					      TRUE);
    localpathPt->valid = is_valid_hilflat(robotPt, 
					  localpathPt->specific.hilflat_data,
					  TRUE);

  }
  localpathPt->ikSol = ikSol;
  return localpathPt;
} 

/*!
 Local planner for Hilare, generating only forward motions
*/
p3d_localpath *p3d_nocusp_hilflat_localplanner(p3d_rob *robotPt, double *qi, 
					       double *qf, int* ikSol)
{
  p3d_localpath *localpathPt;
  configPt q_init, q_cusp, q_end;
  double u_start = PAR_INIT;
  double u_end = PAR_END;
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);

  if (hilflat_params == NULL){
    PrintInfo(("  Local method flat Hilare not initialized\n"));
    return NULL;
  }

  if (p3d_V2(robotPt, qi, qf) <= 0 ) {
    return NULL;
  }

  q_init = p3d_copy_config(robotPt, qi);
  q_cusp = p3d_copy_config(robotPt, qf);
  q_end = NULL;

  localpathPt = p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp,
					    q_end, u_start, u_end, 0,
					    FALSE);
						   
  localpathPt->valid = is_valid_hilflat(robotPt, 
					localpathPt->specific.hilflat_data,
					FALSE);

  if (localpathPt->specific.hilflat_data->init_cusp->v <=0) {
    localpathPt->valid = FALSE;
  }
  localpathPt->ikSol = ikSol;
  return localpathPt;
} 

/*
 *  p3d_create_hilflat_local_method_for_robot --
 *
 *  does same things as p3d_create_hilflat_local_method but for 
 *  given robot
 */
int p3d_create_hilflat_local_method_for_robot(p3d_rob *robotPt, 
					      double *dtab, int *itab)
{
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);

  /* test that hilflat local method has not been already initialized
     for this robot */
  if (hilflat_params != NULL){
    PrintWarning(("  hilflat already initialized\n"));
    return FALSE;
  }

  robotPt->lpl_type = P3D_HILFLAT_PLANNER;
  
  hilflat_params = lm_create_hilflat(robotPt, itab);

  if (hilflat_params != NULL){
    robotPt->local_method_params =
      lm_append_to_list(robotPt->local_method_params, (void*)hilflat_params,
			P3D_HILFLAT_PLANNER);
  }
  return TRUE;
}

/*
 *  p3d_create_hilflat_local_method --
 *
 *  create data-structure to store parameters of hilflat local method 
 *  and precomputed arrays .
 */
int p3d_create_hilflat_local_method(int *itab)
{
  p3d_rob *robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  philflat_str hilflat_params = lm_get_hilflat_lm_param(robotPt);

  /* test that hilflat local method has not been already initialized
     for this robot */
  if (hilflat_params != NULL){
    PrintWarning(("  hilflat already initialized\n"));
    return FALSE;
  }

  robotPt->lpl_type = P3D_HILFLAT_PLANNER;
  
  hilflat_params = lm_create_hilflat(robotPt, itab);

  if (hilflat_params != NULL){
    robotPt->local_method_params =
      lm_append_to_list(robotPt->local_method_params, (void*)hilflat_params,
			P3D_HILFLAT_PLANNER);
  }
  return TRUE;
}

/*
 * Check the joint that must be freezed between x,y,theta
 */
static void unfree_x_y(p3d_jnt *jntPt, int num_theta, int * free_joints)
{
  int i;

  if (jntPt->num>=num_theta) 
    { return; }
  free_joints[jntPt->num] = FALSE;
  for(i=0; i<jntPt->n_next_jnt; i++) {
    unfree_x_y(jntPt->next_jnt[i], num_theta, free_joints);
  }
}


/*
 *  lm_create_hilflat
 *
 *      Set the Steering method at Hilflat
 *      itab[0] and itab[1] are joint ids for base and curvature joint
 */

philflat_str lm_create_hilflat(p3d_rob *robotPt, int *itab)
{
  int i, j, i_curv;
  int dof_x, dof_y, dof_theta, dof_curv;
  int njnt = robotPt->njoints;
  philflat_str hilflat_params = NULL;
  p3d_jnt *jnt_thetaPt, *jnt_xPt, *jnt_yPt, *jnt_curvPt, *jntPt;
  int *free_joints;
  
  /* test that dof exist */
  for(i=0; i<NB_JNT_HILFLAT; i++) {
    if ((itab[i]<0) || (itab[i]>njnt)) {
      PrintWarning(("  lm_create_hilflat: index of joint not valid\n"));
      return NULL;
    }
  }
  if (itab[1]<=itab[0]) {
    PrintWarning((" lm_create_hilflat: false order in the joints\n"));
    return NULL; 
  }
  
  /* check the joint */
  jnt_xPt = jnt_yPt = jnt_thetaPt = robotPt->joints[itab[0]];
  dof_theta = dof_x = dof_y = -1;
  switch(jnt_thetaPt->type) {
  case P3D_ROTATE: /* Check on several joints */
    if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
      PrintWarning(("  lm_create_hilflat: theta not controllable\n"));
      return NULL;
    }
    dof_theta = jnt_thetaPt->index_dof;
    /* Check on previous joint */
    jntPt = jnt_thetaPt->prev_jnt;
    while ((jntPt!=NULL) && (dof_x<0)) { 
      for(i=jntPt->dof_equiv_nbr-1; i>=0; i--) {
	if (p3d_jnt_get_dof_is_user(jntPt, i) && 
	    !p3d_jnt_is_dof_angular(jntPt, i)) {
	  if (dof_y<0) {
	    dof_y = jntPt->index_dof+i;
	    jnt_yPt = jntPt;
	  } else if (dof_x<0) {	 
	    dof_x = jntPt->index_dof+i;
	    jnt_xPt = jntPt;
	  } 
	  else { 
	    break; 
	  }
	}
      }
      jntPt = jntPt->prev_jnt;
    }
    if (dof_x<0) {
      PrintWarning(("  lm_create_hilflat: cannot find x degree of freedom\n"));
      return NULL;
    }
    break;
  case P3D_FREEFLYER:
  case P3D_BASE: /* We use x, y, Rz */
    if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 5) ||
	!p3d_jnt_get_dof_is_user(jnt_thetaPt, 1) ||
	!p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
      PrintWarning(("  lm_create_hilflat: x, y or Rz not controllable\n"));
      return NULL;
    }
    dof_theta = jnt_thetaPt->index_dof+5;
    jnt_yPt = jnt_thetaPt;
    dof_y = jnt_thetaPt->index_dof+1;
    jnt_xPt = jnt_thetaPt;
    dof_x = jnt_thetaPt->index_dof;
    break;
  case P3D_PLAN: /* We use x, y, Rz */
    if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 2) ||
	!p3d_jnt_get_dof_is_user(jnt_thetaPt, 1) ||
	!p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
      PrintWarning(("  lm_create_hilflat: x, y or Rz not controllable\n"));
      return NULL;
    }
    dof_theta = jnt_thetaPt->index_dof+2;
    jnt_yPt = jnt_thetaPt;
    dof_y = jnt_thetaPt->index_dof+1;
    jnt_xPt = jnt_thetaPt;
    dof_x = jnt_thetaPt->index_dof;
    break;
  case P3D_TRANSLATE:
  case P3D_FIXED:
  case P3D_KNEE:
    PrintWarning(("  lm_create_hilflat: theta is not a rotation\n"));
    return NULL;
    break;
  }
  jnt_curvPt  = robotPt->joints[itab[1]];
  dof_curv = -1;
  for(i_curv=jnt_curvPt->dof_equiv_nbr-1; i_curv>=0; i_curv--) {
    if (p3d_jnt_get_dof_is_user(jnt_curvPt, i_curv)) {
      dof_curv = jnt_curvPt->index_dof+i_curv;
      break;
    }
  }
  if (dof_curv<0) {
    PrintWarning(("  lm_create_hilflat: cannot find curvature degree of freedom\n"));
    return NULL; 
  }
  free_joints = MY_ALLOC(int, njnt+1);
  for(i=0; i<=njnt; i++) { 
    free_joints[i] = TRUE; 
  }

  i = 0;
  jntPt = jnt_curvPt;
  do {
    free_joints[jntPt->num] = FALSE;
    jntPt = jntPt->prev_jnt;
    if (jntPt == jnt_xPt) { 
      i++; 
    }
    if (jntPt == jnt_yPt) { 
      i++; 
    }
    if (jntPt == jnt_thetaPt) { 
      i++; 
    }
  } while ((jntPt != NULL) && 
	   ((jntPt->num>jnt_xPt->num) || (jntPt->num>jnt_yPt->num)));
  if ((jntPt == NULL) || (i!=3)) {
    MY_FREE(free_joints, int, njnt+1);
    PrintWarning((" lm_create_hilflat: chaine between phi and x or y is not connected\n"));
    return NULL;     
  }
  free_joints[jntPt->num] = FALSE;
  unfree_x_y(jntPt, jnt_thetaPt->num, free_joints);

  if ((hilflat_params = MY_ALLOC(hilflat_str, 1)) == NULL) {
    MY_FREE(free_joints, int, njnt+1);
    PrintWarning(("  lm_create_hilflat: allocation failed\n"));
    return (NULL);
  }

  hilflat_params->numdof[HILFLAT_DOF_X]       = dof_x;
  hilflat_params->numdof[HILFLAT_DOF_Y]       = dof_y;
  hilflat_params->numdof[HILFLAT_DOF_THETA]   = dof_theta;
  hilflat_params->numdof[HILFLAT_DOF_CURV]    = dof_curv;
  hilflat_params->numjnt[JNT_BASE]    = jnt_thetaPt->num;
  hilflat_params->numjnt[JNT_CURV]    = jnt_curvPt->num;

  hilflat_params->nb_other_jnt = 0;
  for(i=0; i<=njnt; i++) {
    if (free_joints[i]) { 
      hilflat_params->nb_other_jnt ++; 
    }
  }
  hilflat_params->other_jnt = MY_ALLOC(int, hilflat_params->nb_other_jnt);

  /* store dof that are not used by hilflat local method in array 
     other_jnt */
  j = 0;
  for(i=0; i<=njnt; i++) {
    if (free_joints[i]) {
      hilflat_params->other_jnt[j] = i;
      j ++;
    }
  }
  MY_FREE(free_joints, int, njnt+1);
    
  /* p3d_get_robot_jnt_bounds(hilflat_joint, &v_min, &phi_max); DEV KINEO - pas de robot courant, utilise le robot en argument ! */

  return(hilflat_params);
}


/*
 *  lm_get_hilflat_lm_param --
 *
 *  find the first occurence of hilflat local method parameters.
 */

philflat_str lm_get_hilflat_lm_param(p3d_rob *robotPt)
{
  lm_list_param_str *list_paramPt = robotPt->local_method_params;
  philflat_str resultPt=NULL;

  while (list_paramPt) {
    if (list_paramPt->lpl_type != P3D_HILFLAT_PLANNER) {
      list_paramPt = list_paramPt->next;
    }
    else {
      resultPt = (philflat_str)(list_paramPt->lm_param);
      list_paramPt = NULL;
    }
  }
  return resultPt;
}


/*
 *  p3d_read_hilflat_localpath_symmetric --
 *
 * build a symmetric hilflat local path and read the data specific
 * this local path in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 * ARGS OUT : a local path or NULL if error */


p3d_localpath *p3d_read_hilflat_localpath_symmetric(p3d_rob *robotPt, 
						    FILE *file,
						    double version)
{
  p3d_localpath *localpathPt = NULL;
  int size_max_line=0;
  char *pos=NULL, *line=NULL, *name=NULL;
  configPt q_init=NULL, q_cusp=NULL, q_end=NULL;
  int success=TRUE;
  double u_start, u_end;
  int num_line=0;

  static int save_line_size=0;
  static char *save_line=NULL;


  /* 
   *  look for conf_init 
   */

  /* read a line */
  if ((size_max_line = p3d_read_line_next_function(file, &line, 
						   size_max_line, 
						   &num_line)) == 0) {
    PrintWarning(("line %d: expecting initial configuration\n", num_line));
    success=FALSE;
  }
  pos = line;

  if (success) {
    if ((q_init = p3d_read_word_and_config(robotPt, line, 
					      "conf_init", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /* 
   *  look for conf_cusp 
   */
  
  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if ((q_cusp = p3d_read_word_and_config(robotPt, line, 
					   "conf_cusp", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /* 
   *  look for conf_end 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success=FALSE;
    }
    /* copy line to save it */
    save_line = p3d_copy_line(line, save_line, size_max_line);
    save_line_size = size_max_line;
  }
  
  if (success) {
    
    if ((q_end = p3d_read_word_and_config(robotPt, line, 
					  "conf_end", version)) != NULL) {
      /* read next line */
      if ((size_max_line = p3d_read_line_next_function(file, &line, 
						       size_max_line, 
						       &num_line)) == 0) {
	PrintWarning(("line %d: expecting alpha_0\n", num_line));
	success=FALSE;
      }
      pos = line;
    }
    else { 
      /* if no end configuration is specified, restore line */
      line = p3d_copy_line(save_line, line, size_max_line);
      size_max_line = save_line_size;
    }
  }
  /* if no end configuration is specified, do not read next line */

  /* 
   * look for u_start 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting u_start\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if (p3d_read_word_and_double(line, "u_start", &u_start) != TRUE) {
      PrintWarning(("line %d: expecting u_start\n", num_line));
      success=FALSE;
    }
  }

  /* 
   * look for u_end 
   */


  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting u_end\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if (p3d_read_word_and_double(line, "u_end", &u_end) != TRUE) {
      PrintWarning(("line %d: expecting u_end\n", num_line));
      success=FALSE;
    }
  }

  /* 
   * look for p3d_end_local_path 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_string_name(&pos, &name) != TRUE) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
    /* test that first word of line is u_end */
    else if (strcmp(name, "p3d_end_local_path") != 0) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
  }

  if (success) {
    localpathPt = 
      p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp, q_end, 
				  u_start, u_end, 0, TRUE);
  }
  else {
    /* error while readind local path, desallocate and return NULL */
    if (q_init != NULL) {
      p3d_destroy_config(robotPt, q_init);
    }
    if (q_cusp != NULL) {
      p3d_destroy_config(robotPt, q_cusp);
    }
    if (q_end != NULL) {
      p3d_destroy_config(robotPt, q_end);
    }
  }
  return localpathPt;
}


/*
 *  p3d_read_hilflat_localpath_not_symmetric --
 *
 * build a non symmetric hilflat local path and read the data specific
 * this local path in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 * ARGS OUT : a local path or NULL if error */


p3d_localpath *p3d_read_hilflat_localpath_not_symmetric(p3d_rob *robotPt, 
							FILE *file,
							double version)
{
  p3d_localpath *localpathPt = NULL;
  int size_max_line=0;
  char *pos=NULL, *line=NULL, *name=NULL;
  configPt q_init=NULL, q_cusp=NULL, q_end=NULL;
  int success=TRUE;
  double u_start, u_end;
  int num_line=0;

  static int save_line_size=0;
  static char *save_line=NULL;


  /* 
   *  look for conf_init 
   */

  /* read a line */
  if ((size_max_line = p3d_read_line_next_function(file, &line, 
						   size_max_line, 
						   &num_line)) == 0) {
    PrintWarning(("line %d: expecting initial configuration\n", num_line));
    success=FALSE;
  }
  pos = line;

  if (success) {
    if ((q_init = p3d_read_word_and_config(robotPt, line, 
					      "conf_init", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /* 
   *  look for conf_cusp 
   */
  
  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if ((q_cusp = p3d_read_word_and_config(robotPt, line, 
					   "conf_cusp", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /* 
   *  look for conf_end 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success=FALSE;
    }
    /* copy line to save it */
    save_line = p3d_copy_line(line, save_line, size_max_line);
    save_line_size = size_max_line;
  }
  
  if (success) {
    
    if ((q_end = p3d_read_word_and_config(robotPt, line, 
					  "conf_end", version)) != NULL) {
      /* read next line */
      if ((size_max_line = p3d_read_line_next_function(file, &line, 
						       size_max_line, 
						       &num_line)) == 0) {
	PrintWarning(("line %d: expecting alpha_0\n", num_line));
	success=FALSE;
      }
      pos = line;
    }
    else { 
      /* if no end configuration is specified, restore line */
      line = p3d_copy_line(save_line, line, size_max_line);
      size_max_line = save_line_size;
    }
  }
  /* if no end configuration is specified, do not read next line */

  /* 
   * look for u_start 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting u_start\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if (p3d_read_word_and_double(line, "u_start", &u_start) != TRUE) {
      PrintWarning(("line %d: expecting u_start\n", num_line));
      success=FALSE;
    }
  }

  /* 
   * look for u_end 
   */


  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting u_end\n", num_line));
      success=FALSE;
    }
    pos = line;
  }
  
  if (success) {
    if (p3d_read_word_and_double(line, "u_end", &u_end) != TRUE) {
      PrintWarning(("line %d: expecting u_end\n", num_line));
      success=FALSE;
    }
  }

  /* 
   * look for p3d_end_local_path 
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line, 
						     size_max_line, 
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_string_name(&pos, &name) != TRUE) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
    /* test that first word of line is u_end */
    else if (strcmp(name, "p3d_end_local_path") != 0) {
      PrintWarning(("line %d: expecting p3d_end_local_path\n", num_line));
      success=FALSE;
    }
  }

  if (success) {
    localpathPt = 
      p3d_alloc_hilflat_localpath(robotPt, q_init, q_cusp, q_end, 
				  u_start, u_end, 0, FALSE);
  }
  else {
    /* error while readind local path, desallocate and return NULL */
    if (q_init != NULL) {
      p3d_destroy_config(robotPt, q_init);
    }
    if (q_cusp != NULL) {
      p3d_destroy_config(robotPt, q_cusp);
    }
    if (q_end != NULL) {
      p3d_destroy_config(robotPt, q_end);
    }
  }
  return localpathPt;
}


