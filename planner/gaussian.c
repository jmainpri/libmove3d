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
#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
#include "Collision-pkg.h"

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
void p3d_shoot_gaussian(p3d_rob *r, configPt c1, configPt c2)
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
configPt p3d_shoot_gaussian_graph(p3d_rob *r, p3d_graph *G)
{
  int SUCCEED = 0 ;
  configPt c1, c2, NewConfig = NULL;
  int coll_c1,coll_c2;

  c1 = p3d_alloc_config(r);
  c2 = p3d_alloc_config(r);
 
  while(SUCCEED == 0){
    p3d_shoot(r, c1, 1) ; // generates random sample
    p3d_shoot_gaussian(r, c1, c2, 1) ; // generates sample nearby
    
    p3d_set_and_update_robot_conf(c1);
    coll_c1 = p3d_col_test(); // does c1 collide?
    
    p3d_set_and_update_robot_conf(c2);
    coll_c2 = p3d_col_test(); // does c2 collide?
    
    
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
  
  G->nb_q_free = G->nb_q_free + 1; 
  return(NewConfig);
}
