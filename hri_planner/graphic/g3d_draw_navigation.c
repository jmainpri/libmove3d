#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"
#include "Hri_planner-pkg.h"

/*
 * Play the current trajectory of the robot with his bitmap
 *
 * Input:  the robot,
 *
 */

configPt  g3d_bt_dynamic_tshow(p3d_rob *robotPt, int (*fct)(void), int* nb)
{
  double u=0; 
  double du, umax, dmax; /* parameters along the local path */
  configPt q;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0, count=0;
  pp3d_localpath localpathPt;

  if(robotPt->tcur == NULL){
    PrintInfo(("g3d_show_tcur_rob: no current trajectory\n"));
    *nb = 0;
    return NULL;
  }

  localpathPt = robotPt->tcur->courbePt;
  distances = MY_ALLOC(double, njnt+1);

 // AKP : Storing human position before executing path
     //configPt hum_initial_pos = p3d_get_robot_config(BTSET->human[BTSET->actual_human]->HumanPt);
     //printf("AKP : start pos :%lf , %lf ", hum_initial_pos[6], hum_initial_pos[7]); 
 // AKP END

  //dmax = p3d_get_env_graphic_dmax();
  while (localpathPt != NULL){
    umax = localpathPt->range_param;
    
    while (end_localpath < 2){
      dmax = p3d_get_env_graphic_dmax();
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      p3d_set_robot_config(robotPt,q);
      if(BTSET->changed){
	*nb = count;
	MY_FREE(distances, double, njnt+1);
	return q;
      }
      p3d_destroy_config(robotPt, q);
      
      /* collision checking */
      p3d_numcoll = p3d_col_test_all();
      
     // AKP : Checking for human moved
     //configPt hum_cur_pos = p3d_get_robot_config(BTSET->human[BTSET->actual_human]->HumanPt);
     //printf("AKP : start pos :%lf , %lf ", hum_cur_pos[6], hum_cur_pos[7]); 
     //if(hum_cur_pos[6]!=hum_initial_pos[6]||hum_cur_pos[7]!=hum_initial_pos[7])
     //{
     // return(NULL);
     //}
    // AKP END
      
      count++;
      //fprintf(" count = %d ",count);  
      //fflush(stdout);
      
      g3d_draw_allwin_active();
      if(fct) if(((*fct)()) == FALSE){
	*nb = count;
	return(NULL);
      }
      
      for (i=0; i<=njnt; i++){
	distances[i] = dmax;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
  }
  MY_FREE(distances, double, njnt+1);
  *nb = count;
  return NULL;
}
