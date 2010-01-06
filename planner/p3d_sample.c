#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include <locale.h>
#ifdef MULTILOCALPATH
#include "Localpath-pkg.h"
#endif
#define DEBUG(x) x

#include <gsl/gsl_randist.h>
gsl_rng * _gsl_seed;

extern double  InitCostThreshold;

static void p3d_nearby_variation_shoot(p3d_rob *r, configPt q, configPt pc, double maxVar);
static void p3d_binary_search(p3d_rob *r, configPt qInObst,configPt qInFree, double threshold);



/***********************************************/
/* Fonction initialisant le generateur de      */
/* nombres aleatoires par un nombre choisi     */
/* In : l'initialisation                       */
/* Out :                                       */
/***********************************************/
void p3d_init_random_seed(int seed)
{
  srand((unsigned int) (seed));
  _gsl_seed = gsl_rng_alloc (gsl_rng_taus);
}

/***********************************************/
/* Fonction initialisant le generateur de      */
/* nombres aleatoires avec l'horloge           */
/* In :                                        */
/* Out :                                       */
/***********************************************/
void p3d_init_random(void)
{
  srand(time(NULL));
  _gsl_seed = gsl_rng_alloc (gsl_rng_taus);
}

/*******************************************/
/* Fonction generant un nombre aleatoire   */
/* entre a et b                            */
/* Int : a et b                            */
/* Out : le nombre genere                  */
/*******************************************/
double p3d_random(double a, double b)
{double v;

  v =  rand()/((double)RAND_MAX+1); /* nombre aleatoire [0.,1.] */
  v = (b-a)*v + a;
  return(v);
}

/***************************************************/
/* by Boor 7-12-1999                               */
/* Gaussian random number generator                */
/* Given a double Sigma_d_a, NormalRand            */
/* returns a double chosen from a normal           */
/* distribution using sigma Sigma_d_a              */
/***************************************************/
double NormalRand( double Sigma_d_a )
{
  double a,b;
  double result;

  a = p3d_random(.0,1.0);
  // following test necessary only when rand() can return 0!!!
  while(a == 0.0){
    a = p3d_random(.0,1.) ;
  }
  b = p3d_random(.0,1.0);
  result = Sigma_d_a*cos(2.0*M_PI*b)*sqrt(-2.0*log(a));
  return(result);
}

/* modif Juan Cortes */
/*************************************************/
/* Fonction generant une configuration aleatoire */
/* pour un robot                                 */
/*************************************************/
int p3d_standard_shoot(p3d_rob *robotPt, configPt q, int sample_passive)
{
  int njnt = robotPt->njoints, i, j, k;
  double vmin, vmax;
  p3d_jnt * jntPt;
  
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;

      if (p3d_jnt_get_dof_is_user(jntPt, j) &&
	  (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) || sample_passive)) {
	p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	q[k] = p3d_random(vmin, vmax);
//        std::cout << "Sample Passive = "<<sample_passive<<" , Sampling q["<<k<<"] = "<<q[k]<< std::endl;
      } else
	{ q[k] = p3d_jnt_get_dof(jntPt, j); }
    }
  }
#ifdef  MULTILOCALPATH
  p3d_localplanner_type lpl_type = robotPt->lpl_type;
  if (lpl_type == P3D_MULTILOCALPATH_PLANNER) {
    configPt qTmp = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
    configPt qTmp2;
    for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
      if(p3d_multiLocalPath_get_value_groupToPlan(robotPt, i)){
        qTmp2 = p3d_separateMultiLocalPathConfig(robotPt, qTmp, q , i, robotPt->mlp->mlpJoints);
        p3d_copy_config_into(robotPt, qTmp2, &qTmp);
        p3d_destroy_config(robotPt,qTmp2);
      }
    }
    p3d_copy_config_into(robotPt, qTmp, &q);
    p3d_destroy_config(robotPt,qTmp);
  }
#endif

 return(TRUE);
}
/***************************************************/
/* Function which creates randomly a configuration */
/* for the robot inside a specific box             */
/***************************************************/
static int p3d_inbox_shoot(p3d_rob *robotPt,  configPt box_env[], configPt q, int sample_passive)
{
  double vmin, vmax;
  double s1,s2, rand;
  int njnt = robotPt->njoints, i, j, k; 
  p3d_jnt * jntPt;
  
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      // TEST
/*       p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax); */
/*       if() { */

/*       } */
      ////////
      if (p3d_jnt_get_dof_is_user(jntPt, j) &&
	  (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) || sample_passive)) {
	p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	if (p3d_jnt_is_dof_angular(jntPt, j)) 
	  {
	    if(vmin >= -M_PI && vmax <= M_PI) {
	      if((box_env[0][k] <-M_PI) || (box_env[0][k] > M_PI) ||
		 (box_env[1][k] <-M_PI) || (box_env[1][k] > M_PI) )
		{ PrintInfo(("Warning :Wrong bounds of shooting box\n"));}
	    }	
	    if(box_env[0][k] <= box_env[1][k]) 
	      {
		q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	      }
	    else {
	      //we have to shoot randomly in 
	      //the union of the 2 intervals [-Pi, max]U[min, Pi]
	      s1 =  box_env[1][k]+M_PI;
	      s2= M_PI- box_env[0][k];
	      rand = p3d_random(0.,s1+s2);
	      if(rand <s1) //we are in the second interval 
		{
		  q[k] = -M_PI+rand; 
		}
	      else {
		q[k] = box_env[0][k] + rand - s1;
	      }
	    }
	  }
	else {
	  q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	}
	// if (vmin < box_env[0][k]) vmin = box_env[0][k]; 
	// 	if (vmax > box_env[1][k]) vmax = box_env[1][k]; 
	// 	q[k] = p3d_random(vmin, vmax); 
      } else
	{ q[k] = p3d_jnt_get_dof(jntPt, j); }
    }
  }
  return(TRUE);
}


/***************************************************/
/* by Boor 7-12-1999                               */
/* p3d_shoot_gaussian(Robot, Config, Config)       */
/* Gaussian nearby configuration generator         */
/* in: the robot, the starting configuration,      */
/*     a nearby configuration                      */
/* out: void                                       */
/* uses: p3d_random_gaussian(Sigma, JointType)     */
/*   Generates a configuration c2 that is "close   */
/*   to" (given a certain Normal distribution) the */
/*   starting configuration c1                     */
/*   Ensures that c2 is inside the workspace.      */
/***************************************************/
static
void p3d_gaussian_config2(p3d_rob *r, configPt c1, configPt c2, int sample_passive)
{
  int njnt = r->njoints, ij; 
  double robotsize, jmin, jmax, sig = 10.0;
  int i, k;
  p3d_jnt * jntPt;

  /* TODO(?): somehow reflect robot size. 
   * Something you would not want
   * to have to compute every time,
   * but just once when reading the file.
   * It would be nicer to hace a robot.size() member 
   *inside the p3d_rob struct... */
  p3d_get_BB_rob_max_size(r, &robotsize); /* new Carl 23052001 */
  for(ij=0;ij<=njnt;ij++){
    jntPt = r->joints[ij];
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {  
      k = jntPt->index_dof+i;
      if (p3d_jnt_get_dof_is_user(jntPt, i) &&
	  (p3d_jnt_get_dof_is_active_for_planner(jntPt,i) || sample_passive)) {
	p3d_jnt_get_dof_rand_bounds(jntPt, i, &jmin, &jmax);
	if (p3d_jnt_is_dof_angular(jntPt, i)) {
	  /* ugly hack: you would want the distribution to
	     be dependent on the used distance measure... */
	  sig = 30.0; 
	} else
	  { sig = 10.0; }
	if (fabs(jmax - jmin) > EPS6) {
	  do {
	    c2[k] = c1[k] + NormalRand(robotsize/sig);
	  } while((c2[k] < jmin) || (c2[k] > jmax));
	}
      }
      else {
	c2[k] = c1[k];
      }
    }
  }
}

/***************************************************/
/* adapted by Boor 7-12-1999                       */
/* p3d_shoot_gaussian_graph                        */
/* in: the robot and the graph                     */
/* out: a configuration                            */
/* uses: p3d_shoot_gaussian(r, c1, c2),            */
/*       p3d_shoot(r, c1)                          */
/*  Generates a new RobotConfig using the Gaussian */
/*  Sampling method                                */
/***************************************************/
static int
p3d_gaussian_shoot(p3d_rob *r, configPt q, int sample_passive)
{
  int SUCCEED = 0 ;
  configPt c1, c2, NewConfig = NULL;
  int coll_c1,coll_c2;

  c1 = p3d_alloc_config(r);
  c2 = p3d_alloc_config(r);
 
  while(SUCCEED == 0){
    p3d_standard_shoot(r, c1, sample_passive) ; // generates random sample
    p3d_gaussian_config2(r, c1, c2, sample_passive) ; // generates sample nearby
    
    p3d_set_and_update_this_robot_conf_without_cntrt(r,c1);
    coll_c1 = p3d_col_test(); // does c1 collide?
    (r->GRAPH->nb_test_coll)++;//Mokhtar
    
    p3d_set_and_update_this_robot_conf_without_cntrt(r,c2);
    coll_c2 = p3d_col_test(); // does c2 collide?
    (r->GRAPH->nb_test_coll)++; //Mokhtar
    
    
    if ( !coll_c1 ){
      // c1 is a free configuration
      if ( !coll_c2 ){
  // c2 is a free configuration
        SUCCEED = 0 ;
      }
      else{        
        // c2 is a forbidden configuration
        NewConfig = c1 ;
        SUCCEED = 1 ;
      }
    }
    else{          // c1 is a forbidden configuration
      if ( !coll_c2 ){
  // c2 is a free configuration
        NewConfig = c2 ;
        SUCCEED = 1 ;
      }
      else{        // c2 is a forbidden configuration
        SUCCEED = 0 ;
      }
    }
  }
  p3d_copy_config_into(r,NewConfig,&q);
  p3d_destroy_config(r,c1);
  p3d_destroy_config(r,c2);
  return(TRUE);
}

/*************************************************/
/* Fonction generant une configuration avec      */
/* tirage aleatoire standard ou en utilisant     */
/* Halton sets                                   */
/* Cette fonction utilisse le RLG dans le cas    */
/* de chaines fermees                            */ 
/* NOTE:                                         */
/* the "sample_passive" flag does not work with Halton*/
/*************************************************/
int p3d_shoot(p3d_rob *robotPt, configPt q, int sample_passive)
{ p3d_graph *G;
  int go_on;
  //  int nFail = 0;

  G = robotPt->GRAPH;
  go_on = 1;
  while(go_on) {
    if (p3d_get_RANDOM_CHOICE()==P3D_HALTON_SAMPLING)
      p3d_getNextHHpoint(robotPt, &(G->hhCount), q, sample_passive);
    else
      /* Modif Mokhtar */
      //new
      switch (p3d_get_SAMPLING_CHOICE()){
        case P3D_GAUSSIAN_SAMPLING :{
          p3d_gaussian_shoot(robotPt,q,sample_passive);
          break;
        }
        case P3D_UNIFORM_SAMPLING :{
          p3d_standard_shoot(robotPt,q,sample_passive);
          break;
        }
        case P3D_BRIDGE_SAMPLING :{
          p3d_bridge_shoot(robotPt,q,sample_passive);
          break;
        }
        case P3D_OBPRM_SAMPLING :{
           p3d_obprm_shoot(robotPt,q,sample_passive);
          break;
        }
      }
      /* Fin Modif Mokhtar */
      if (p3d_get_RLG()) {
        if (p3d_random_loop_generator(robotPt, q)) {
          go_on = 0;
        }
      } else {
        go_on = 0;
      }

/*     if(p3d_GetIsCostFuncSpace() && p3d_GetIsCostThreshold() ) { */
/*       if(p3d_GetConfigCost(robotPt, q) < p3d_GetCostThreshold()) { */
/* 	go_on = MAX(go_on, 0); */
/*       } else if (nFail > 500) { */
/* 	PrintInfo(("Failed 500 times to find an expansion configuration\ */
/* with a good cost. Return a random configuration"));  */
/* 	p3d_SetCostThreshold(p3d_GetCostThreshold() + InitCostThreshold); */
/* 	go_on = 0; */
/*       } */
/*       else { */
/* 	nFail++; */
/* 	go_on = 1; */
/*       } */
/*     } */
    if(G != NULL)
      G->nb_q = G->nb_q + 1; 
  }
  
  return(TRUE);
}

/*************************************************/
/* Fonction generant une configuration a         */
/* l'interieur d'une boite englobante  avec      */
/* tirage aleatoire standard ou en utilissant    */
/* Halton sets                                   */
/* Cette fonction utilisse le RLG dans le cas    */
/* de chaines fermees (leonard :a revoir...)     */
/*************************************************/
int p3d_shoot_inside_box(p3d_rob *robotPt, configPt q, configPt box_env_small[2], int sample_passive)
{ p3d_graph *G;
  int go_on;
 
  G = robotPt->GRAPH;
  go_on = 1;
  while(go_on) {    
    if (p3d_get_RANDOM_CHOICE()==P3D_HALTON_SAMPLING)
      p3d_getNextHHpoint_for_q_in_box(robotPt,box_env_small, &(G->hhCount), q, sample_passive);
    else
      if(p3d_get_SAMPLING_CHOICE() == P3D_GAUSSIAN_SAMPLING)
      	{	
      	  p3d_gaussian_shoot(robotPt,q,sample_passive);
	  PrintInfo(("Warning :  we no more use a box for the shooting method \n"));
      	}
      else
        {
	  p3d_inbox_shoot(robotPt,box_env_small, q, sample_passive);
      }
      if(p3d_get_RLG()) {
  if(p3d_random_loop_generator(robotPt,q)) {
    go_on = 0;
  }
      }
      else {
  go_on = 0;
      }
      G->nb_q = G->nb_q + 1; 
      //modif leonard : use projection for the direction of expension
      //if(p3d_get_is_projection == TRUE) {
      //p3d_set_and_update_this_robot_conf(robotPt, q);
      //}
  }
  
  return(TRUE);
}

/****************************************************************************/
/** \brief Generates a new RobotConfig using the bridge test Sampling method
 \param *r pointer to the robot
 \param q Configuration pointer
 \param sample_passive flag for the passive joints
 */
/****************************************************************************/
void p3d_bridge_shoot(p3d_rob *r, configPt q, int sample_passive){
   int SUCCEED = 0;
   configPt c1, c2, middle;

   c1 = p3d_alloc_config(r);
   c2 = p3d_alloc_config(r);
   middle = p3d_alloc_config(r);
   while(SUCCEED == 0){
      p3d_standard_shoot(r, c1, sample_passive) ; // generates random sample
      p3d_set_and_update_this_robot_conf_without_cntrt(r,c1);
      if (p3d_col_test()){// does c1 collide?
         (r->GRAPH->nb_test_coll)++;
         p3d_nearby_variation_shoot(r,c2,c1,0.1);
         p3d_set_and_update_this_robot_conf_without_cntrt(r,c2);
         if (p3d_col_test()){// does c2 collide?
            (r->GRAPH->nb_test_coll)++;
            p3d_middleConfig(r,c1,c2,middle);

            p3d_set_and_update_this_robot_conf_without_cntrt(r,middle);
            if (!p3d_col_test()){
               (r->GRAPH->nb_test_coll)++;
               SUCCEED = 1;
               p3d_copy_config_into(r,middle,&q);
            }
         }
      }
   }
  p3d_destroy_config(r,c1);
  p3d_destroy_config(r,c2);
  p3d_destroy_config(r,middle);
}

/****************************************************************************/
/** \brief Generates a new Robot Config but don't exceed maxVar
 \param *r pointer to the robot
 \param q Configuration pointer
 \param pc previous configuration pointer
 \param maxVar max variation around pc
 */
/****************************************************************************/
static void p3d_nearby_variation_shoot(p3d_rob *r, configPt q, configPt pc, double maxVar){
  p3d_jnt * jntPt;
  int i, j, k,  njnt = r->njoints;
  double vmin, vmax, d;
  for(i=0; i<=njnt; i++){
    jntPt = r->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      p3d_jnt_get_dof_rand_bounds(jntPt,j,&vmin,&vmax);
      d = vmax - vmin; 
      q[k] = p3d_random(MAX(pc[k]-d*maxVar, vmin), MIN(pc[k]+d*maxVar,vmax));
    }
  }
}
/****************************************************************************/
/** \brief Generates a new RobotConfig using the OBPRM Sampling method 
 \param *r pointer to the robot
 \param q Configuration pointer
 \param sample_passive flag for the passive joints
 */
/****************************************************************************/
void p3d_obprm_shoot(p3d_rob *r, configPt q, int sample_passive){
  int i, j, k, njnt = r->njoints;
  double vmin, vmax, l, variation = 1, binThreshold = 0.0;
  p3d_jnt * jntPt;
  configPt first;
  
  //random shoot, don't stop until colision.
  first = p3d_alloc_config(r);
  do{
    p3d_standard_shoot(r, first, sample_passive) ; // generates random sample
    p3d_set_and_update_this_robot_conf_without_cntrt(r,first);
  }while(!p3d_col_test());
  // générer des configurations autour de cette configuration
  // une configuration n'est retenue qu'une fois qu'elle n'est pas en colision
  // une fois la configuration générér, rapprocher par recherche binaire cette
  // configuration du bord de l'obstacle
    do{
      for(i=0;i<=njnt;i++){
        jntPt = r->joints[i];
        for(j=0; j<jntPt->dof_equiv_nbr; j++) {
          binThreshold = MAX(binThreshold, l);
          k = jntPt->index_dof+j;
          p3d_jnt_get_dof_rand_bounds(jntPt,j,&vmin,&vmax);
          l = vmax-vmin;
          q[k] = p3d_random(MAX(first[k]-variation*l,vmin),MIN(first[k]+variation*l,vmax));
        }
      }
      p3d_set_and_update_this_robot_conf_without_cntrt(r,q);
    }while(p3d_col_test());
    binThreshold *= 0.1;
    p3d_binary_search(r,first,q,binThreshold);
}

/****************************************************************************/
/** \brief Do a binary search between a free and collision configuration
 \param *r pointer to the robot
 \param qInObst Configuration in collision pointer
 \param qInFree Configuration out of collision pointer
 \param threshold threshold of the binary search
 */
/****************************************************************************/
static void p3d_binary_search(p3d_rob *r, configPt qInObst,configPt qInFree, double threshold){
  configPt first, qMiddle;
  int i = 0;

  qMiddle = p3d_alloc_config(r);
  first = p3d_alloc_config(r);
  p3d_copy_config_into(r,qInObst,&first);

  while (p3d_dist_config(r,first,qInFree) > threshold && i < 20){

    p3d_middleConfig(r,first,qInFree,qMiddle);
    p3d_set_and_update_this_robot_conf_without_cntrt(r,qMiddle);
    if(!p3d_col_test()){//Cfree
      p3d_copy_config_into(r,qMiddle,&qInFree);
    }else{//Cobs
      p3d_copy_config_into(r,qMiddle,&first);
    }
    (r->GRAPH->nb_test_coll)++;
    i++;
  }
  p3d_destroy_config(r,qMiddle);
  p3d_destroy_config(r,first);
}


/**
 * p3d_RandDirShoot
 * Shoot a pseudo-random direction
 * (still better than space sampling)
 * this direction is not normalized !
 * @param[In] robotPt: the current robot
 * @param[out] q: the configuration as random direction
 * @param[In] sample_passive: TRUE if the passive dofs have to
 * be taken into consideration
 * @return: Currently, it returns always TRUE has a new 
 * direction can always success, but it could change.
 */
int p3d_RandDirShoot(p3d_rob* robotPt, configPt q, int sample_passive) {
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax;
  p3d_jnt * jntPt;
  
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;

      if (p3d_jnt_get_dof_is_user(jntPt, j) &&
	  (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) || sample_passive)) {
	p3d_jnt_get_dof_rand_bounds(jntPt,j,&vmin,&vmax);
	if(p3d_jnt_is_dof_angular(jntPt, j)) {
	  q[k] = p3d_random(0, 2*M_PI);
	}
	else {
	  q[k] = p3d_random(-(vmax-vmin)/2.,(vmax-vmin)/2.);
	}
      }
    }
  }
  return TRUE;
}


/**
 * p3d_RandNShpereDirShoot

 * @param[In] robotPt: the current robot
 * @param[out] q: the configuration as random direction
 * @param[In] sample_passive: TRUE if the passive dofs have to
 * be taken into consideration
 * @return: Currently, it returns always TRUE has a new
 * direction can always success, but it could change.
 */

int p3d_RandNShpereDirShoot(p3d_rob* robotPt, configPt q, int sample_passive)
{
	int njnt = robotPt->njoints, i, j, k;
	double vmin, vmax;
	p3d_jnt * jntPt;

	size_t dim = robotPt->nb_user_dof;

	int id_num=0;
	int *id_vect = new int[dim];
	double *dir = new double[dim];

	gsl_ran_dir_nd (_gsl_seed,dim,dir);

	/*for(int i=0;i<dim;i++)
	{
		printf("dir[%d] = %f\n",i,dir[i]);
	}*/

	for (i = 0; i <= njnt; i++)
	{
		jntPt = robotPt->joints[i];
		for (j = 0; j < jntPt->dof_equiv_nbr; j++)
		{
			k = jntPt->index_dof + j;

			if (p3d_jnt_get_dof_is_user(jntPt, j)
					&& (p3d_jnt_get_dof_is_active_for_planner(jntPt, j)
							|| sample_passive))
			{
				/*if (p3d_jnt_is_dof_angular(jntPt, j))
				{
					double b = 0;
					double a = M_PI;
					q[k] = a*dir[id_num]+b;
				}
				else
				{*/
					p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
					double a = (vmax - vmin) / 2.;
					double b = (vmax + vmin) / 2.;
					q[k] = a*dir[id_num]+b;
//				}
				id_vect[id_num] = k;
				id_num++;
			}
		}
	}

	/*for(int i=0;i<dim;i++)
	{
		printf("q[%d] = %f\n",id_vect[i],q[id_vect[i]]);
	}

	printf("---------------------\n");*/

	return TRUE;
}

/**
 * isOutOfBands
 */

bool p3d_isOutOfBands(p3d_rob* robotPt, configPt q, int sample_passive)
{
	int njnt = robotPt->njoints, i, j, k;
	double vmin, vmax;
	p3d_jnt * jntPt;

	for (i = 0; i <= njnt; i++)
	{
		jntPt = robotPt->joints[i];
		for (j = 0; j < jntPt->dof_equiv_nbr; j++)
		{
			k = jntPt->index_dof + j;

			if (p3d_jnt_get_dof_is_user(jntPt, j)
					&& (p3d_jnt_get_dof_is_active_for_planner(jntPt, j)
							|| sample_passive))
			{
				/*if (p3d_jnt_is_dof_angular(jntPt, j))
				{
					if( q[k]<(-M_PI) || q[k]>M_PI)
					{
						return true;
					}
				}
				else
				{*/
					p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);

					if( q[k]<vmin || q[k]>vmax)
					{
						return true;
					}
//				}
			}
		}
	}

	return false;
}


/**
  * Sample the freeFlyerOnly inside a box
  * box = xmin,xmax,ymin,ymax,zmin,zmax
  */
void p3d_FreeFlyerShoot(p3d_rob* robotPt, configPt q, double* box )
{
    for (int i = 0; i <= robotPt->njoints; i++)
    {
        p3d_jnt* jntPt = robotPt->joints[i];

        if( jntPt->type != P3D_FREEFLYER )
        {
            continue;
        }
        else
        {
//            std::cout << "Joint " << i << " is a free flyer" << std::endl;
        }

        for (int j = 0; j < jntPt->dof_equiv_nbr; j++)
        {
            int k = jntPt->index_dof + j;

            if (p3d_jnt_get_dof_is_user(jntPt, j) && (p3d_jnt_get_dof_is_active_for_planner(jntPt, j)))
            {
                if (p3d_jnt_is_dof_angular(jntPt, j))
                {
                    double vmax,vmin;
                    p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
                    q[k] = p3d_random(vmin,vmax);
                }
                else
                {
                    q[k] = p3d_random(box[2*j],box[2*j+1]);
                }
            }
        }
    }
}
