#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h" 
#include "Hri_planner-pkg.h"

#ifndef PROTO
#ifdef UNIX
#include "GL/glx.h"
#include "forms.h"
#endif
#endif
#include "glcanvas.h"
static psp_lst_vertex lstvert;
//static int  vertIndex;

static double lx1, ly1, lz1, lx2, ly2, lz2,lrad;
//static double ox=-1.8,oy=-0.57,oz=0.59;
static double ox=0.0,oy=-0.0,oz=0.0;
static int sphereActive=0;
static psp_searchball srchball;

static hri_gik * PSP_GIK  = NULL;
static hri_gik * PSP_GIK2 = NULL;
static hri_gik * PSP_GIK3 = NULL;

static configPt theqs[20];
static int qindex=0;
static int  PSP_DRAW_QS = FALSE;

//static int PSP_GIK_CONTINUOUS = TRUE;
p3d_rob *PSP_ROBOT;
int PSP_DEACTIVATE_AUTOHIDE = 1;
/*******************************************************************************************************************************************************/

/* --------  Perspective Placement functions --------*/

/* List of points generator functions*/
static void     psp_gen_rand_point                    (p3d_vector4 rpoint,double  xMaxLim, double xMinLim, double  yMaxLim, double yMinLim);
static void     psp_gen_rand_3Dpoint                  (p3d_vector4 rpoint, p3d_vector4 center, double Rhomin, double Rhomax);
static void     psp_gen_ordered_point_list            (p3d_rob *obr, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_gen_ordered_point_list_obj        (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_gen_ordered_point_list_searchball (psp_searchball *sball, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_gen_ordered_spheric_point_list    (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_order_point_list                  (psp_lst_vertex *lstVtx);
static int      psp_get_next_ordered_point            (p3d_vector4 rpoint, psp_lst_vertex *lstVtx);

/* Inverse Kinematics and Configuration test functions */
static int      psp_look_at              (p3d_rob* r, double x, double y, double z, configPt* resq);
static int      psp_look_in_two_times_at (p3d_rob* r, double fromx, double fromy, double fromz, double tox, double toy, double toz, configPt* resq);
static int      psp_test_qs              (p3d_rob *r, configPt q1, configPt q2, p3d_vector4 point , double viewPercent, int checkTraj, hri_bitmapset* PSP_BTSET);

/* Searching Placement Point Functions*/
int  psp_rnd_srch_model_pt       (p3d_rob* r, p3d_rob* objRob, int numpoints, hri_bitmapset* PSP_BTSET);
int  psp_srch_model_pt           (p3d_rob* r, p3d_rob* objRob, int numpoints, hri_bitmapset* PSP_BTSET);
int  psp_srch_3D_model_pt_obj    (p3d_rob* r, p3d_obj* object, int numpoints, int OnSurface, hri_bitmapset* PSP_BTSET);
int  psp_srch_model_pt_obj       (p3d_rob* r, p3d_obj* object, int numpoints, double viewPercent, hri_bitmapset* PSP_BTSET);
int  psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r, int numpoints, hri_bitmapset* PSP_BTSET);
int  psp_srch_for_target_obj     (p3d_rob *robot, int numPoints, int searchMode, hri_bitmapset* PSP_BTSET);

/* --------- Perspective Observation functions ---------*/

static double pso_watch2_obj();
static double pso_watch_obj();
static int    pso_see_obj();
static int    pso_look_obj();
static double pso_perceive_obj();

/* --------- Perspective Reasoning functions  ---------*/ // Testing at 23/08/07

static int    psr_human_look_at (p3d_rob* human, double x, double y, double z);
static double psr_preference_obj(p3d_vector4 obj, double disttocenter, p3d_vector4 porigin);

void psr_get_obj_list();
void psr_get_human_left_pointing             (p3d_rob* human, p3d_rob* r, hri_bitmapset* PSP_BTSET);
void psr_get_human_pointing_from_joint_number(p3d_rob* human, p3d_rob* r, int jntIdx, hri_bitmapset* PSP_BTSET);
void psr_get_pointing_from_joint             (p3d_rob* r,  p3d_jnt *jntPt, int frameType, hri_bitmapset* PSP_BTSET);


/* --------- Utility  Section -------------- */

/* Testing and interface */ 
void  psp_draw_test             ();
void  psp_draw_search_ball      (psp_searchball *srchballpt);
void  psp_draw_in_perspwin      ();
void  psp_search_for_objectives (p3d_rob *robot, p3d_vector3 point);
void  psp_add_element           (psp_lst_elements *lstel, psp_obs_element *elem);
void  g3d_psp_draw_lookatpoint  (p3d_rob *robot);


/* Selection Robot and objects functions */
void p3d_select_robot_to_view    (p3d_rob *robotPt);
void p3d_deselect_robot_to_view  (p3d_rob *robotPt);
int  p3d_get_rob_select_status   (p3d_rob *robotPt);
void p3d_deselect_all_objects    ();
void p3d_select_object_to_view   (p3d_obj *objectPt);
void p3d_unselect_object_to_view (p3d_obj *objectPt);
int  p3d_get_obj_select_status   (p3d_obj *objectPt);
void p3d_set_body_selection      (p3d_rob *r, int body, int val );



/* Getting objects and robots Center functions */ 
static double p3d_get_vertical_center               (p3d_rob* rob);
static double p3d_get_obj_centertoborder_distance   (p3d_obj* obj);
static double p3d_get_robot_centertoborder_distance (p3d_rob *r);

void   p3d_get_robot_center                         (p3d_rob* rob, p3d_vector4 pointc);
void   p3d_get_object_center                        (p3d_obj* obj, p3d_vector4 pointc);



/* tool functions */ 
double p3d_psp_pointtolinedist    (p3d_vector3 p, p3d_vector3 l1, p3d_vector3 l2);
void   p3d_psp_cartesian2spherical(double x, double y, double z,
				   double originx, double originy, double originz,
				   double *phi, double *theta);

void   p3d_psp_spherical2cartesian(double x, double y, double z, 
				   double rad, double phi, double theta, 
				   p3d_vector4 point);

int    p3d_psp_is_point_in_a_cone (p3d_vector4 p, p3d_vector4 conep, 
				   p3d_vector3 conep2  , double coneangle, double *distf);



int    p3d_init_robot_parameters();
int    psp_init_bitmap_grids();
int    p3d_init_object_parameters_by_name    (char *objName,  double min, double max);
int    p3d_init_all_object_parameters_by_type(char *objType,  double min, double max);


/*******************************************************************************************************************************************************/



/****************************************************************/
/*!
 * \brief Find a 2D random point  inside an rectalgular area
 * 
 * \param rpoint - resulting point
 * \param xMaxLim, xMinLim - Limits in X axe
 * \param yMaxLim, yMinLim - Limits in Y axe
 * !

 */
/****************************************************************/

static void psp_gen_rand_point(p3d_vector4 rpoint,double  xMaxLim, double xMinLim, double  yMaxLim, double yMinLim)
{
  //double lowest=0, highest=10; 
  double rangex=(xMaxLim-xMinLim),rangey=(yMaxLim-yMinLim); 
  rpoint[0] = xMinLim+(rangex*rand()/(RAND_MAX + 1.0)); 
  rpoint[1] = yMinLim+(rangey*rand()/(RAND_MAX + 1.0));
  rpoint[3] = 1.0;
}


/****************************************************************/
/*!
 * \brief Find a 3D random point around a point
 * 
 * \param rpoint resulting point
 * \param center point around which we search
 * \param Rhomin minimum distance to center point 
 * \param Rhomax maximum distance to center point
 * !

 */
/****************************************************************/
static void psp_gen_rand_3Dpoint(p3d_vector4 rpoint, p3d_vector4 center, double Rhomin, double Rhomax)
{
  double diffRho = (Rhomax-Rhomin);
  double theta = M_2PI * rand()/(RAND_MAX + 1.0) ;
  double phi   = M_PI * rand()/(RAND_MAX + 1.0) ;
   
  /* spherical representation of a point on a sphere */ 
  rpoint[0] = (Rhomin+diffRho*rand()/(RAND_MAX + 1.0)) * cos(theta) * sin(phi) + center[0]; 
  rpoint[1] = (Rhomin+diffRho*rand()/(RAND_MAX + 1.0)) * sin(theta) * sin(phi) + center[1]; 
  rpoint[2] = (Rhomin+diffRho*rand()/(RAND_MAX + 1.0)) * cos(theta) + center[2]; 
  rpoint[3] = 1.0;
  
}


/****************************************************************/
/*!
 * \brief Orders by cost the point list
 * 

 * !

 */
/****************************************************************/

static void  psp_order_point_list (psp_lst_vertex *lstVtx)
{
  int i,j;
  double costaux;
  int statusaux,idaux, swapped;
  p3d_vector4 rvertex;

    for (i=lstVtx->nv-1; --i>=0;) 
      {
	swapped = 0;
	for (j=0; j<i;j++) 
	  {
	    if (lstVtx->vertex[j].cost > lstVtx->vertex[j+1].cost) 
	      {
		idaux      = lstVtx->vertex[j].id;
	        rvertex[0] = lstVtx->vertex[j].pos[0];
		rvertex[1] = lstVtx->vertex[j].pos[1];
		rvertex[2] = lstVtx->vertex[j].pos[2];
		costaux    = lstVtx->vertex[j].cost;
		statusaux  = lstVtx->vertex[j].status;


		lstVtx->vertex[j].id     = lstVtx->vertex[j+1].id;
		lstVtx->vertex[j].pos[0] = lstVtx->vertex[j+1].pos[0];
		lstVtx->vertex[j].pos[1] = lstVtx->vertex[j+1].pos[1];
		lstVtx->vertex[j].pos[2] = lstVtx->vertex[j+1].pos[2];
		lstVtx->vertex[j].cost   = lstVtx->vertex[j+1].cost;
		lstVtx->vertex[j].status = lstVtx->vertex[j+1].status;


		lstVtx->vertex[j+1].id     = idaux;
		lstVtx->vertex[j+1].pos[0] = rvertex[0];
		lstVtx->vertex[j+1].pos[1] = rvertex[1];
		lstVtx->vertex[j+1].pos[2] = rvertex[2];
		lstVtx->vertex[j+1].cost   = costaux;
		lstVtx->vertex[j+1].status = statusaux;

		swapped = 1;
		
	      }
	  }
	if (!swapped) return;
      }

}







/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost
 * 
 * \param or -> objetctif 
 * \param r  -> Robot 
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/




static void     psp_gen_ordered_point_list (p3d_rob *obr, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = ((obr->max_pos_range -  obr->min_pos_range) / 2.0) + obr->min_pos_range ; 
  p3d_vector4 auxpoint, rvertex;
  p3d_jnt *jntPt = obr->joints[1]; 
  p3d_matrix4 matrix;
  
  double angle;
  int i,j,x,y,z=0;
  int contVert = 0;
 
  double maxAngle, maxDist = DISTANCE2D(jntPt->p0.x ,jntPt->p0.y,r->joints[1]->p0.x ,r->joints[1]->p0.y) + distance;
  double percCost1, percCost2, btcost;

  double gainDist= 0.6;

  lstVtx->nv = numpoints;
  lstVtx->currentVert = 0;

  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      matrix[i][j]=jntPt->abs_pos[i][j];
    }
  }

  auxpoint[3]  = 1.0;
  matrix[2][3] = 0.0;

  maxAngle = numpoints/2;
 
  for (i=0; i<=numpoints/2; i++)
    {
      angle  = (2*M_PI)*i/(numpoints*2); 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      p3d_matvec4Mult(matrix,auxpoint,rvertex); //obtaining global coordinates

      ////////////// Costs 
           
      //cost = distace to the point + distance from direct front position (bigger the angle bigger the distance)
      //we should add social and security bitmap cost
      percCost1 =  (DISTANCE2D(r->joints[1]->abs_pos[0][3], r->joints[1]->abs_pos[1][3],  rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  (i * (1-gainDist)) / maxAngle;

      percCost1 = percCost1 + percCost2;

      if (p3d_is_in_pos_area (obr,auxpoint[0],auxpoint[1], FALSE))
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	}
      else
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	  percCost1 +=6.0;
	}

      //cost of bitmap 
      x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
      y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;
 
      if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2)
	{
	  btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
	  if (btcost == -2)
	    {
	      btcost = 800;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	    }
	  else
	    if (btcost == -1)
	      {
		btcost = 100;
		lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	      }
	    else
	      {
		btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
		lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE; 
	      }
	}
      else
	{
	  btcost = 800;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}

      //cost 50% of each cost calculation
      percCost1 = (percCost1 * 0.7)/10.0;
      percCost2 = (btcost * 0.3)/430.0;

      lstVtx->vertex[contVert].cost   =  percCost1 + percCost2;


      ///////////////////

      //printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }

  //maxAngle = numpoints*2 - (numpoints*1.5);

  for (i+=numpoints; i<numpoints*2; i++)
    {
      angle  = (2*M_PI)*i/(numpoints*2); 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      p3d_matvec4Mult(matrix,auxpoint,rvertex); //obtaining global coordinates


      ////////////// Costs 


      //cost = distace to the point + distance from direct front position (smaller the angle bigger the distance)
      percCost1 =  (DISTANCE2D(r->joints[1]->abs_pos[0][3], r->joints[1]->abs_pos[1][3],  rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  (1-gainDist) * ((maxAngle - (i - (numpoints*1.5))) / maxAngle);

      percCost1 = percCost1 + percCost2;

      if (p3d_is_in_pos_area (obr,auxpoint[0],auxpoint[1], FALSE))
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	}
      else
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	  percCost1 +=6.0;
	}

      //cost of bitmap 
      x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
      y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

      if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2)
	{
	  btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
	  if (btcost == -2)
	    {
	      btcost = 800;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	    }
	  else
	    if (btcost == -1)
	      {
		btcost = 100;
		lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	      }
	    else
	      {
		btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
		lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE; 
	      }
	}
      else
	{
	  btcost = 800;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}

      //cost 50% of each cost calculation
      percCost1 = (percCost1 * 0.7)/10.0;
      percCost2 = (btcost * 0.3)/430.0;

      lstVtx->vertex[contVert].cost   =  percCost1 + percCost2;

      ///////////////////

      //printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }

  psp_order_point_list(lstVtx);
  //printf("List Ordered\n");
}


/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost
 * 
 * \param or -> objetctif 
 * \param r  -> Robot 
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_ordered_point_list_wtraj (p3d_rob *obr, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = ((obr->max_pos_range -  obr->min_pos_range) / 2.0) + obr->min_pos_range ; 
  p3d_vector4 auxpoint, rvertex;
  p3d_jnt *jntPt = obr->joints[1]; 
  p3d_matrix4 matrix;

  double ps[3], pg[3];  

  double angle;
  int i,j,x,y,z=0;
  int contVert = 0;
 
  double maxAngle, maxDist = DISTANCE2D(jntPt->p0.x ,jntPt->p0.y,r->joints[1]->p0.x ,r->joints[1]->p0.y) + distance;
  double percCost1, percCost2, btcost;

  double gainDist= 0.2;

  lstVtx->nv = numpoints;
  lstVtx->currentVert = 0;

  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      matrix[i][j]=jntPt->abs_pos[i][j];
    }
  }

  auxpoint[3]  = 1.0;
  matrix[2][3] = 0.0;

  ps[0] = r->joints[1]->abs_pos[0][3];
  ps[1] = r->joints[1]->abs_pos[1][3];
  ps[2] = r->joints[1]->abs_pos[2][3];//0.0;

  pg[2] = r->joints[1]->abs_pos[2][3];//0.0;

  maxAngle = numpoints/2;
 
  for (i=0; i<=numpoints/2; i++)
    {
      angle  = (2*M_PI)*i/(numpoints*2); 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      p3d_matvec4Mult(matrix,auxpoint,rvertex); //obtaining global coordinates

      ////////////// Costs 

      
      //cost = distace to the point + distance from direct front position (bigger the angle bigger the distance)
      //we should add social and security bitmap cost
      percCost1 =  (DISTANCE2D(ps[0],ps[1],  rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  (i * (1-gainDist)) / maxAngle;

      percCost1 = percCost1 + percCost2;

      if (p3d_is_in_pos_area (obr,auxpoint[0],auxpoint[1], FALSE))
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	}
      else
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	  percCost1 +=6.0;
	}

      //cost of bitmap 

      pg[0] = rvertex[0];
      pg[1] = rvertex[1];


      hri_bt_reset_path(PSP_BTSET);
      if (  (btcost =  hri_bt_start_search(ps,pg, PSP_BTSET, 0)) < 0)
	{
	  btcost = 100;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}
      //cost 50% of each cost calculation
      
      percCost1 *= 0.10;
      percCost1 *= 0.5;
            
      percCost2 = btcost * .8;
      percCost2 *= 0.5; 

      lstVtx->vertex[contVert].cost   =  percCost1 + percCost2 ;


      ///////////////////

      printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }

  //maxAngle = numpoints*2 - (numpoints*1.5);

  for (i+=numpoints; i<numpoints*2; i++)
    {
      angle  = (2*M_PI)*i/(numpoints*2); 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      p3d_matvec4Mult(matrix,auxpoint,rvertex); //obtaining global coordinates


      ////////////// Costs 


      //cost = distace to the point + distance from direct front position (smaller the angle bigger the distance)
      percCost1 =  (DISTANCE2D(ps[0],ps[1], rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  (1-gainDist) * ((maxAngle - (i - (numpoints*1.5))) / maxAngle);

      percCost1 = percCost1 + percCost2;

      if (p3d_is_in_pos_area (obr,auxpoint[0],auxpoint[1], FALSE))
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	}
      else
	{
	  lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
	  percCost1 +=6.0;
	}


      pg[0] = rvertex[0];
      pg[1] = rvertex[1];

      hri_bt_reset_path(PSP_BTSET);
      if (  (btcost =  hri_bt_start_search(ps,pg, PSP_BTSET, 0)) < 0)
	{
	  btcost = 100;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}
      //cost 50% of each cost calculation
      
      percCost1 *= 0.10;
      percCost1 *= 0.5;
            
      percCost2 = btcost * .8;
      percCost2 *= 0.5; 

      //cost 50% of each cost calculation
      //percCost1 = (percCost1 * 0.7)/10.0;
      //percCost2 = (btcost * 0.3)/430.0;

      lstVtx->vertex[contVert].cost   =  percCost1 + percCost2;

      ///////////////////

      printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }

  psp_order_point_list(lstVtx);
  //printf("List Ordered\n");
}



/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost for an object
 * 
 * \param object -> object
 * \param r  -> Robot 
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_ordered_point_list_obj (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = ((object->max_pos_range -  object->min_pos_range) / 2.0) + object->min_pos_range ; 
  p3d_vector4 auxpoint, rvertex, objCenter;


  double ps[3], pg[3];  
  
  double angle;
  int i,x,y,z=0;
  int contVert = 0;
   
  p3d_get_object_center(object, objCenter);

  double maxAngle, maxDist = DISTANCE2D(objCenter[0], objCenter[1], r->joints[1]->p0.x, r->joints[1]->p0.y);
  double percCost1, percCost2, btcost;

  double gainDist= 0.6;

  ps[0] = r->joints[1]->abs_pos[0][3];
  ps[1] = r->joints[1]->abs_pos[1][3];
  ps[2] = r->joints[1]->abs_pos[2][3];//0.0;

  pg[2] = r->joints[1]->abs_pos[2][3];//0.0;
 
  lstVtx->nv = numpoints;
  lstVtx->currentVert = 0;

  maxAngle = numpoints/2;
 
  for (i=0; i<=numpoints; i++)
    {
      angle  = (2*M_PI)*i/numpoints; 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      //obtaining global coordinates
      rvertex[0] = auxpoint[0] + objCenter[0]; 
      rvertex[1] = auxpoint[1] + objCenter[1]; 
      rvertex[2] = 0.0;
      rvertex[3] = 1.0;
      ////////////// Costs 

      
      //cost = distace to the point + distance from direct front position (bigger the angle bigger the distance)
      //we should add social and security bitmap cost
      percCost1 =  (DISTANCE2D(ps[0], ps[1],  rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  0.0;//Meter distancia de robot o cantidad de percepcion (i * (1-gainDist)) / maxAngle;

      percCost1 = percCost1 + percCost2;
 

      x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
      y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

      if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2)
	{
	  btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
	  if (btcost < -1)
	    {
	      btcost = 800;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      printf("in obstacle\n");
	    }
	  else
	    if (btcost == -1)
	      {
		btcost = 300;
		lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
		printf("high cost\n");
	      }
	    else
	      {
		btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
		lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      }
	  printf("btcost %f\n",btcost);

	}
      else
	{
	  btcost = 800;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}


/*
      pg[0] = rvertex[0];
      pg[1] = rvertex[1];
      btcost =  hri_bt_start_search(ps,pg, PSP_BTSET, 0);

      //printf("%f \n", btcost);
      if ( btcost  < 0.0)
	{
	 // printf("paso \n");

	  btcost = 100;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}
*/
      //cost 30% - 70% for correspondant costs
      
      percCost1 *= 0.10;
      percCost1 *= 0.3;
            
      percCost2 = btcost * 4.3;
      percCost2 *= 0.7; 


      lstVtx->vertex[contVert].cost  =   percCost1 + percCost2;


      ///////////////////

      printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }


  //printf("List Ordered\n");
  psp_order_point_list(lstVtx);

}




/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost for an object
 * 
 * \param object -> object
 * \param r  -> Robot 
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_ordered_point_list_searchball (psp_searchball *sball, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = ((sball->distMax - sball->distMin ) / 2.0) + sball->distMin; 
  p3d_vector4 auxpoint, rvertex;

  
  double angle;
  int i,x,y,z=0;
  int contVert = 0;
   

  double maxAngle, maxDist = DISTANCE2D(sball->position[0], sball->position[1], r->joints[1]->p0.x, r->joints[1]->p0.y);
  double percCost1, percCost2, btcost;

  double gainDist= 0.6;

  
  lstVtx->nv = numpoints;
  lstVtx->currentVert = 0;

  maxAngle = numpoints/2;
 
  for (i=0; i<=numpoints; i++)
    {
      angle  = (2*M_PI)*i/numpoints; 
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      auxpoint[2] = 0.0;
      
      
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      //obtaining global coordinates
      rvertex[0] = auxpoint[0] + sball->position[0]; 
      rvertex[1] = auxpoint[1] + sball->position[1]; 
      rvertex[2] = 0.0;
      rvertex[3] = 1.0;
      ////////////// Costs 

      
      //cost = distace to the point + distance from direct front position (bigger the angle bigger the distance)
      //we should add social and security bitmap cost
      percCost1 =  (DISTANCE2D(r->joints[1]->abs_pos[0][3], r->joints[1]->abs_pos[1][3],  rvertex[0], rvertex[1])*gainDist) / maxDist;
      percCost2 =  0.0;//Meter distancia de robot o cantidad de percepcion (i * (1-gainDist)) / maxAngle;

      percCost1 = percCost1 + percCost2;


      //cost of bitmap 
      x = (rvertex[0] - BTSET->realx)/ BTSET->pace;
      y = (rvertex[1] - BTSET->realy)/ BTSET->pace;

      if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2)
	btcost = BTSET->bitmap[BT_COMBINED]->calculate_cell_value(BTSET,x,y,z);
      else
	{
	  btcost = 800;
	  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	}

      //cost 50% of each cost calculation
      percCost1 = (percCost1 * 0.7)/10.0;
      percCost2 = (btcost * 0.3)/430.0;

      lstVtx->vertex[contVert].cost   =  percCost1 + percCost2;


      ///////////////////

      //printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
      contVert++;
    }



  psp_order_point_list(lstVtx);

}






/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost for an object
 * 
 * \param object -> object
 * \param r  -> Robot 
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_ordered_spheric_point_list (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = ((object->max_pos_range -  object->min_pos_range) / 2.0) + object->min_pos_range ; 
  p3d_vector4 auxpoint, rvertex, objCenter;

  
  double angle,angle2;
  int i,j,x,y,z=0;
  int contVert = 0;
  
  p3d_get_object_center(object, objCenter);

  double maxAngle;// maxDist = DISTANCE2D(objCenter[0], objCenter[1], r->joints[1]->p0.x, r->joints[1]->p0.y);
  double percCost1, percCost2, btcost = 0.0;

  double gainDist= 0.6;
  

  ox = objCenter[0]; 
  oy = objCenter[1]; 
  oz = objCenter[2]; 
 
  
  printf("min pos %f max pos %f distance %f\n",object->min_pos_range,object->max_pos_range, distance);
  
  if (distance>0)
    {
      //lstVtx->nv = numpoints;
      lstVtx->currentVert = 0;

  
      maxAngle = numpoints;
 
      for (i=1; i<=numpoints; i++)
	{
	  angle  = (M_PI)*i/numpoints; 

	  for (j=1; j<=maxAngle; j++)
	    {   
	      btcost = 0.0;
	      angle2  = (2*M_PI)*j/numpoints; 
	      auxpoint[0] = distance * sin(angle) * cos(angle2);
	      auxpoint[1] = distance * sin(angle) * sin(angle2);
	      auxpoint[2] = distance * cos(angle);
	      if (auxpoint[2]>=0)
		{	  
		  lstVtx->vertex[contVert].pos[0] = auxpoint[0];
		  lstVtx->vertex[contVert].pos[1] = auxpoint[1];
		  lstVtx->vertex[contVert].pos[2] = auxpoint[2];

		  //obtaining global coordinates
		  rvertex[0] = auxpoint[0] + objCenter[0]; 
		  rvertex[1] = auxpoint[1] + objCenter[1]; 
		  rvertex[2] = auxpoint[2] + objCenter[2]; 
		  rvertex[3] = 1.0;
		  ////////////// Costs 

      
		  //cost = distace to the point + distance from direct front position (bigger the angle bigger the distance)
		  //we should add social and security bitmap cost
		  percCost1 =  (DISTANCE3D(r->joints[r->cam_body_index]->abs_pos[0][3], r->joints[r->cam_body_index]->abs_pos[1][3], 
					   r->joints[r->cam_body_index]->abs_pos[2][3], rvertex[0], rvertex[1],rvertex[2])*gainDist);

		  
		  if (DISTANCE3D(rvertex[0], rvertex[1], rvertex[2], 
				 r->joints[5]->abs_pos[0][3], 
				 r->joints[5]->abs_pos[1][3],
				 r->joints[5]->abs_pos[2][3]) > 1.2)
		    {
		      btcost = 450;
		      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
		    }
		  else
		    {
		      lstVtx->vertex[contVert].status =  PSP_St_OBSERVABLE;

		      x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
		      y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;
		  
		      if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2)
			btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
		      else
			{
			  btcost = 450;
			  lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
			}
		    }

		  percCost1 *= 0.10;
		  percCost1 *= 0.3;
            
		  percCost2 = btcost * 4.3;
		  percCost2 *= 0.7; 



		  //cost 50% of each cost calculation
		  //percCost1 = (percCost1 * 0.7)/10.0;
		  //percCost2 = (btcost * 0.3)/430.0;

		  lstVtx->vertex[contVert].cost   =  percCost1 + percCost2;
		  ///////////////////
	    
		    printf("%f %f %f %f\n",auxpoint[0] ,auxpoint[1], auxpoint[2], lstVtx->vertex[contVert].cost );
		    contVert++;	  
		}
	    }


	}

      //printf("List Ordered\n");
      lstVtx->nv = contVert;
      psp_order_point_list(lstVtx);
    }
  else
    printf("Distance must be greater that 0 (SET OBTJECT PARAMETERS FIRST)\n");

}






/****************************************************************/
/*!
 * \brief Find the next point in the list of points
 * 
 * \param rpoint resulting point

 * !

 */
/****************************************************************/

static int psp_get_next_ordered_point(p3d_vector4 rpoint, psp_lst_vertex *lstVtx )
{

  if(lstVtx->vertex[lstVtx->currentVert].status==PSP_St_NOT_IN_RANGE)
    {
      if (lstVtx->currentVert >= lstVtx->nv)
	lstVtx->currentVert = 0;
      return 0;
    }

  rpoint[0] = lstVtx->vertex[lstVtx->currentVert].pos[0];
  rpoint[1] = lstVtx->vertex[lstVtx->currentVert].pos[1];
  rpoint[2] = lstVtx->vertex[lstVtx->currentVert].pos[2];  
  rpoint[3] = 1.0;

  lstVtx->currentVert++; 

  if (lstVtx->currentVert >= lstVtx->nv)
    lstVtx->currentVert = 0;
  return 1;
}



/****************************************************************/
/*!
 * \brief Finds a robot configuration to look to a point
 * 
 * \param r - robot
 * \param xo, yo, zo - point
 * \param resq - resulting q
 * !
 */
/****************************************************************/

static int psp_look_at(p3d_rob* r, double x, double y, double z, configPt* resq)
{

  //double mintmp = r->joints[32]->dof_data[0].vmin;

#ifdef JIDO
  int jointInd= ROBOTj_LOOK;
#endif
#ifdef BH 
  int jointInd= 32;
#endif

  p3d_vector3 point2look;

  int res=0;

#ifdef JIDO
  int jointindexesR[]= {ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK}; //Jido (Platine1,platine2, look)
#endif
#ifdef BH 
  int jointindexesR[]= {2,4,5,6,32}; //BH (body, neck, head1, head2, head3, look)
#endif

  point2look[0]=x;
  point2look[1]=y;
  point2look[2]=z;

  if (PSP_GIK != NULL)
    {
      if(!PSP_GIK->GIKInitialized){
#ifdef JIDO
	/***** FOR JIDO *****/
	hri_gik_initialize_gik(PSP_GIK,r,1,3); //
	hri_gik_add_task(PSP_GIK, 3, 3, 1, jointindexesR, ROBOTj_LOOK);  /* Cameras */
#endif
#ifdef BH
	/***** FOR BH *****/    
	hri_gik_initialize_gik(PSP_GIK,r,1,5); /* Attention to joint number */
	hri_gik_add_task(PSP_GIK, 3, 5, 1, jointindexesR, ROBOTj_LOOK);  /* HEAD */
#endif
      }

      printf("INITIALIZED\n");
      res = hri_gik_compute(r, PSP_GIK, 200, 0.1, 1, 0, &point2look,NULL, resq, NULL);

      if (res)
	{  
	  p3d_set_and_update_this_robot_conf(r,*resq);
	  printf("CONF. FOUND\n");
	}
      else
	{
	  //resq = NULL;
	  printf("NO CONF.FOUND\n");
	}
    }


  return(res);

}


/****************************************************************/
/*!
 * \brief Finds a robot configuration to look to a point but in two times
 * 
 * \param r - robot
 * \param xo, yo, zo - point
 * \param resq - resulting q
 * !
 */
/****************************************************************/

static int psp_look_in_two_times_at(p3d_rob* r, double fromx, double fromy, double fromz, double tox, double toy, double toz, configPt* resq)
{


  p3d_vector3 point2look;
  p3d_vector4 jointcenter;

  int res=0;

  int jointindexesR2[]= {5, 6, 7, 8, 9, 10}; //Jido arm
  int jointindexesR3[]= {8, 9, 10, ROBOTj_POINT}; //Jido arm with look

  lx1=point2look[0]=fromx;
  ly1=point2look[1]=fromy;
  lz1=point2look[2]=fromz;

  p3d_get_object_center(r->joints[10]->o, jointcenter);
  
  printf("Point to reach %f %f %f \n", fromx,fromy,fromz);
  printf("Point of joint %f %f %f \n", jointcenter[0], jointcenter[1], jointcenter[2]);

  if (PSP_GIK2 != NULL && PSP_GIK3 != NULL)
    {
      printf("INITIALIZING ... \n");
      if(!PSP_GIK2->GIKInitialized){
	/***** FOR JIDO *****/
	hri_gik_initialize_gik(PSP_GIK2,r,1,6); //
	hri_gik_add_task(PSP_GIK2, 3, 6, 1, jointindexesR2, 10);  /* Placement */
      }
      printf("INITIALIZED 1 \n");
      if(!PSP_GIK3->GIKInitialized){
	/***** FOR JIDO *****/
	hri_gik_initialize_gik(PSP_GIK3,r,1,4); 
	hri_gik_add_task(PSP_GIK3, 3, 4, 1, jointindexesR3, ROBOTj_POINT);  /* Orientation */
      }

      printf("INITIALIZED 2 \n");
      res = hri_gik_compute(r, PSP_GIK2, 200, 0.1, 1, 0, &point2look, NULL, resq, NULL);
    
      printf("First conf found \n");
      if (res)
	{  
	  p3d_set_and_update_this_robot_conf(r,*resq);
	  point2look[0]=tox;
	  point2look[1]=toy;
	  point2look[2]=toz;
	  printf("Trying second \n");
	  res = hri_gik_compute(r, PSP_GIK3, 200, 0.1, 1, 0, &point2look, NULL, resq, NULL);

	  if (res)
	    {  
	      p3d_set_and_update_this_robot_conf(r,*resq);
	      p3d_get_object_center(r->joints[10]->o, jointcenter);
	      lx2=jointcenter[0];
	      ly2=jointcenter[1];
	      lz2=jointcenter[2];
	      printf("Last point of joint %f %f %f \n", jointcenter[0], jointcenter[1], jointcenter[2]);
	      printf("CONF. FOUND\n");
	      return (res);
	    }
	}

      printf("NO CONF.FOUND\n");
    }


  return(res);

}

/****************************************************************/
/*!
 * \brief test if  proposed configuration is valid
 * 
 * \param r - robot
 * \param q1, q2 - 
 * \param resq - resulting q
 * !
 */
/****************************************************************/

static int psp_test_qs(p3d_rob *r, configPt q1, configPt q2, p3d_vector4 point, double viewPercent , int checkTraj, hri_bitmapset* PSP_BTSET)
{
  int res=0;
  double qs[3], qg[3];
  double kcd_with_report;
  //printf("------------------- Setting Robot---------------------\n");
  res = p3d_set_and_update_this_robot_conf(r,q1);
  //printf("------------------- Drawing windows---------------------\n");  
  g3d_draw_allwin_active();
  //g3d_refresh_allwin_active();
  ////movie stuff
  //g3d_save_movie_image();
  g3d_refresh_allwin_active();
      
  if (res) // is a valid configuration?
    {
      res = 0;
      //printf("------------------- Turning head ---------------------\n");
      if (psp_look_at(r, point[0], point[1], point[2], &q1))//Can turn its head to center point?
	{
	  res = p3d_col_test_robot(r,kcd_with_report);
	  //g3d_refresh_allwin_active();
	  if(!res) // isn't there a collision?  ------ p3d_col_test_choice(); p3d_col_env_set_traj_method(type); test_current_coll_call_times();
	    {
	      //printf("------------------- Whatching ---------------------\n");
	      if (pso_watch2_obj()>=viewPercent)//pso_see_obj()) // can the robot see the object?
		{
		  //printf("------------------- coping config ---------------------\n");
		  p3d_copy_config_into(r,q1, &(r->ROBOT_GOTO));
		  p3d_copy_config_into(r,q2, &(r->ROBOT_POS));
		  res = 1;
		  if (checkTraj)
		    {  
		      res =0;
		      if (DISTANCE2D(q1[ROBOTq_X],q1[ROBOTq_Y],q2[ROBOTq_X],q2[ROBOTq_Y])==0)
			res =1;
		      else
			{
			  //printf("------------------- checking Btset ---------------------\n");
			  if(PSP_BTSET!=NULL){  
			    hri_bt_reset_path(PSP_BTSET); 
			  }
			  if(BTGRAPH!=NULL){
			    p3d_del_graph(BTGRAPH);
			    BTGRAPH = NULL;
			  }
			  res = 0;
			  //glFlush();
			  //printf("------------------- Calculating Path ---------------------\n");
			  //res =  hri_bt_calculate_bitmap_path(PSP_BTSET, r , r->ROBOT_POS, r->ROBOT_GOTO, FALSE);
			  qs[0] = r->ROBOT_POS[ROBOTq_X];
			  qs[1] = r->ROBOT_POS[ROBOTq_Y];
			  qs[2] = 0.0;//r->ROBOT_POS[ROBOTq_Z];
			  qg[0] = r->ROBOT_GOTO[ROBOTq_X];
			  qg[1] = r->ROBOT_GOTO[ROBOTq_Y];
			  qg[2] = 0.0;//r->ROBOT_GOTO[ROBOTq_Z];

			  res =  hri_bt_start_search(qs,qg, PSP_BTSET, 0);
			  //printf("res: %i \n",res);
			}
		    }
		  //glFlush();
		  if (res) // is there a valid path?

		    {
		      p3d_set_and_update_this_robot_conf(r,q2);
		      printf("------------------- Modeling Point Found ---------------------\n");
		      p3d_destroy_config(r, q1);
		      p3d_destroy_config(r, q2);
		      g3d_refresh_allwin_active();
		      //g3d_end_movie();
		      return TRUE;
		    }
		  else
		    printf("not valid path\n");

		}
	      else
		printf("Not visible \n");
	    }
	  else
	    {
	      printf("Point in collision \n");
	    }
	}
      else
	printf("I can't look objectif\n");
    }
  else
    printf("not valid conf. /n");

 //g3d_refresh_allwin_active();
  return FALSE;
}



/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of a human camera oriented to the objectif
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/
int psp_test_destination_to_robot(p3d_rob *r, p3d_rob *objRob, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point, v_aux;
  configPt qcurr, qaux, objqcurr;
  //int i;

  qcurr    = p3d_get_robot_config(r);
  qaux     = p3d_get_robot_config(r); 

  p3d_copy_config_into(r,r->ROBOT_GOTO,&qaux);

  objqcurr = p3d_get_robot_config(objRob);

  v_aux[0] = qaux[ROBOTq_X] - objqcurr[HUMANq_X];
  v_aux[1] = qaux[ROBOTq_Y] - objqcurr[HUMANq_Y];
  v_aux[2] = 0.0;

  if (p3d_is_in_pos_area(objRob,v_aux[0],v_aux[1],FALSE))
    {      
      p3d_get_robot_center(objRob, point);
      if(psp_test_qs(r, qaux, qcurr,point,80.0,1, PSP_BTSET))
	{
	  printf("I'm in a good place and I won't move\n"); 	  
	  p3d_destroy_config(r,objqcurr);
	  return TRUE;
	}
    }

}


/****************************************************************/
/*!
 * \brief test current robot conf for viewing the objectif
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/
int psp_test_actual_robot_pos(p3d_rob *r, p3d_rob *objRob, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point, v_aux;
  configPt qcurr, qaux, objqcurr;
  //int i;
  p3d_jnt *jntPt = objRob->joints[1];
  double x,y,xo,yo,zo;
  double refHumAngle,refHumAngle2,refHumAngle3,dist ;
 
  qcurr    = p3d_get_robot_config(r);
  qaux     = p3d_get_robot_config(r); 

  //p3d_copy_config_into(r,r->ROBOT_GOTO,&qaux);

  objqcurr = p3d_get_robot_config(objRob);


  point[0] = 1.0;
  point[1] = 0.0;
  point[2] = 0.0;
  point[3] = 1.0;
  
  p3d_matvec4Mult(objRob->joints[1]->abs_pos,point,v_aux);
  
  refHumAngle2 = rad_angleOf(objqcurr[HUMANq_X],objqcurr[HUMANq_Y],v_aux[0],v_aux[1]);  
  refHumAngle3 = rad_angleOf(objqcurr[HUMANq_X],objqcurr[HUMANq_Y],qcurr[ROBOTq_X],qcurr[ROBOTq_Y]);

  v_aux[0] = qaux[ROBOTq_X] - objqcurr[HUMANq_X];
  v_aux[1] = qaux[ROBOTq_Y] - objqcurr[HUMANq_Y];
  v_aux[2] = 0.0;
  v_aux[3] = 1.0;
//sphereActive =1;
  dist = DISTANCE2D(qcurr[ROBOTq_X],qcurr[ROBOTq_Y],objqcurr[HUMANq_X], objqcurr[HUMANq_Y]);

  //if (p3d_is_in_pos_area(objRob,v_aux[0],v_aux[1],FALSE))
  if (dist<= objRob->max_pos_range && dist >= objRob->min_pos_range)
    {      
      //printf("1 - in the area %f --> %f - %f = %f\n", dist, refHumAngle2,refHumAngle3, refHumAngle2-refHumAngle3 );
      if ( refHumAngle3-refHumAngle2<=M_PI/2 || refHumAngle3-refHumAngle2>=(M_PI/2)*3)
	{
	  p3d_get_robot_center(objRob, point);
	  ox = point[0];
	  oy = point[1];
	  oz = zo = point[2];
	  point[2] = zo + (objRob->joints[55]->abs_pos[2][3]-zo)/1.5;

	  //printf("in the area %f\n", dist ); 
	  if(psp_test_qs(r, qaux, qcurr,point,80.0,0, PSP_BTSET))
	    {
	      if(!p3d_equal_config(r, qcurr, qaux))
		{
		  printf("Good position but change configuration\n"); 
		  p3d_destroy_config(objRob, objqcurr);
		  return 1;
		}
	      printf("I'm in a good configuration and I won't move\n"); 	  
	      p3d_destroy_config(objRob, objqcurr);
	      return 2;
	    }
	  else
	    {
	      p3d_set_and_update_this_robot_conf(r,qcurr);	
	      p3d_destroy_config(r, qaux);
	      qaux     = p3d_get_robot_config(r); 
	      refHumAngle = rad_angleOf(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);
	  
	      if (zo < 0.65 && DISTANCE2D(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y])<2)//check this conditional 
		{
		  if (refHumAngle3>3*(M_PI/2) && refHumAngle2<M_PI/2)
		    {
		      refHumAngle3 -= 2*M_PI;
		    }
		  if ( refHumAngle3 - refHumAngle2 < 0 )
		    refHumAngle -= .7;
		  else
		    refHumAngle += .7;
		}
	      
	      qaux[ROBOTq_RZ] = refHumAngle;
	      //printf("Angle %f\n",refHumAngle); 
	     
	      if(psp_test_qs(r, qaux, qcurr,point,80.0,1, PSP_BTSET))
		{
		  printf("Changing orientation\n"); 	  
		  p3d_destroy_config(objRob, objqcurr);
		  return 3;	      
		}
	    }
	}
    }
  p3d_destroy_config(r, qcurr);
  p3d_destroy_config(r, qaux);
  p3d_destroy_config(objRob, objqcurr);
  printf("Out of zone\n"); 	  
  return 0;
}


/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of a human camera oriented to the objectif
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/



int psp_srch_model_pt(p3d_rob* r, p3d_rob* objRob, int numpoints, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point, point2, v_aux, humHead;
  p3d_jnt *jntPt = objRob->joints[1];

  double refHumAngle, refHumAngle2, refHumAngle3;
  double x,y,xo,yo,zo;
  configPt qcurr, qaux,objqcurr;
  int i;

  PSP_DRAW_QS = FALSE;
// double f_min, f_max;
//  p3d_jnt *jntPt2;
//  int i_DoF;

  //jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_RZ+1,&i_DoF);
  //p3d_jnt_get_dof_bounds(jntPt2,i_DoF, &f_min, &f_max);


  qcurr    = p3d_get_robot_config(r);
  qaux     = p3d_get_robot_config(r); 
  objqcurr = p3d_get_robot_config(objRob);
  point[0] = qaux[ROBOTq_X];
  point[1] = qaux[ROBOTq_Y];
  point[2] = 0.0;
  point[3] = 1.0;
  
  v_aux[0] = qaux[ROBOTq_X] - objqcurr[HUMANq_X];
  v_aux[1] = qaux[ROBOTq_Y] - objqcurr[HUMANq_Y];
  v_aux[2] = 0.0;

/*  if (p3d_is_in_pos_area(objRob,v_aux[0],v_aux[1],FALSE))
    {
      p3d_get_robot_center(objRob, point);
      if(psp_test_qs(r, qaux, qcurr,point))
      {
	  printf("I'm in a good place and I won't move\n"); 

	  p3d_destroy_config(r,objqcurr);
	  return TRUE;
	  }
    }*/


  point[0] = 1.0;
  point[1] = 0.0;
  point[2] = 0.0;
  point[3] = 1.0;
  
  p3d_matvec4Mult(objRob->joints[1]->abs_pos,point,v_aux);

  refHumAngle2 = rad_angleOf(objqcurr[ROBOTq_X],objqcurr[ROBOTq_Y],v_aux[0],v_aux[1]);  


  printf(" Human Angle %f\n",refHumAngle2);
  point[0] = 0.0;
  p3d_get_robot_center(objRob, point2);
  ox = xo = point2[0];
  oy = yo = point2[1];
  zo = point2[2];
  //printf("x -> %f , %f\n",x,xo);
  //printf("y -> %f , %f\n",y,yo);
  //sphereActive =1;
  oz = point2[2] = zo + (objRob->joints[55]->abs_pos[2][3]-zo)/1.5;	    
  printf("z -> %f \n",zo);


  point[0] = 0.0;
  point[1] = 0.0;
  point[2] = 0.0;
  point[3] = 1.0;

  v_aux[0] = 0.0;
  v_aux[1] = 0.0;
  v_aux[2] = 0.0;

  qindex=0;

  //lastAngleCam  = qaux[ROBOTq_PAN];
 
  //jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_PAN,&i_DoF);

  psp_gen_ordered_point_list (objRob, r, numpoints, &lstvert, PSP_BTSET);    
  p3d_set_and_update_this_robot_conf(r,qcurr);	
  for (i=0;i<numpoints;i++)
    {

      if(psp_get_next_ordered_point(point,&lstvert))
	{

	  printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      
	  // passing from local human  to global coords
	  p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);

	  x = v_aux[0];
	  y = v_aux[1];
	  
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////	  
	    qaux[ROBOTq_X] = x;
	    qaux[ROBOTq_Y] = y;


	    ///Horizontal robot angle
	      refHumAngle = rad_angleOf(x,y,xo,yo);

	      refHumAngle3 = rad_angleOf(xo,yo,x,y);
/* 	      lastAngle =  rad_angleOf(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y]); */
/* 	      if (lastAngle>2*M_PI) */
/* 		lastAngle -=2*M_PI; */
	      printf("Angles %f --- %f \n",refHumAngle2,refHumAngle3);
	      if (zo < 0.65 && DISTANCE2D(x,y,xo,yo)<2)//check this conditional 
		{
		  if (refHumAngle3>3*(M_PI/2) && refHumAngle2<M_PI/2)
		    {
		      refHumAngle3 -= 2*M_PI;
		    }
		  
		  if ( refHumAngle3 - refHumAngle2 < 0 )
		    refHumAngle -= .7;
		  else
		    refHumAngle += .7;		  
		}
	      
	      //printf("Angle  %f max %f\n", refHumAngle,f_max);
	      qaux[ROBOTq_RZ] = refHumAngle;
	      p3d_copy_config_into(r,qaux, &theqs[qindex]);
	      
	      //printf("Angles the qs %f --- %f \n",qaux[ROBOTq_RZ],theqs[qindex][ROBOTq_RZ]);
	      qindex++;
//	      p3d_set_and_update_this_robot_conf(r,qaux);
//	      g3d_refresh_allwin_active();
	      if(psp_test_qs(r, qaux, qcurr,point2,80.0,1, PSP_BTSET))
		{
		  p3d_destroy_config(r,objqcurr);
		  //g3d_draw_allwin_active();
		  PSP_DRAW_QS = TRUE;
		  return TRUE;
		}
	}
      else
	printf("Point %i out of range\n",i);
    }

  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(r,qcurr);	
  
  printf("---- ERROR: Not modeling point found -----\n");
  
  p3d_destroy_config(r,qaux);
  p3d_destroy_config(r,objqcurr);
  p3d_destroy_config(r,qcurr);
  g3d_refresh_allwin_active();

  //g3d_end_movie();
  PSP_DRAW_QS = TRUE;
  return FALSE;
}



/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of an object in the environment
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/


int psp_srch_model_pt_obj(p3d_rob* r, p3d_obj* object, int numpoints, double viewPercent, hri_bitmapset* PSP_BTSET)//firsttime and surface and with looking or not
{
  p3d_vector4 point,objCenter;
  double refObjAngle;
  double x,y,xo,yo,zo;
  configPt qcurr, qaux;
  int i;

  p3d_get_object_center(object, objCenter);
  
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r); 

  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  


  psp_gen_ordered_point_list_obj (object, r, numpoints,&lstvert, PSP_BTSET);    

  for (i=0;i<numpoints;i++)
    {

      if(psp_get_next_ordered_point(point,&lstvert))
	{

	  printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      
	  // passing from local to global coords
	  

	  x = point[0]+objCenter[0];
	  y = point[1]+objCenter[1];
	  //point[2] = point[2]+objCenter[2];
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////	  
	    qaux[ROBOTq_X] = x;
	    qaux[ROBOTq_Y] = y;

	    xo = objCenter[0];
	    yo = objCenter[1];
	    zo = objCenter[2];
	    //printf("x -> %f , %f\n",x,xo);
	    //printf("y -> %f , %f\n",y,yo);
	    //printf("z -> %f \n",zo);
     
	    ///Horizontal robot angle
	      refObjAngle = rad_angleOf(x,y,xo,yo);
	      
	      //printf("Angle  %f", refObjAngle);
	      qaux[ROBOTq_RZ] = refObjAngle;

	      if(psp_test_qs(r, qaux, qcurr,objCenter,viewPercent,1, PSP_BTSET))
		return TRUE;
	}
      else
	printf("Point %i out of range\n",i);
    }

  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(r,qcurr);	

  printf("---- ERROR: Not modeling point found -----\n");
  
  p3d_destroy_config(r,qaux);
  p3d_destroy_config(r,qcurr);

  g3d_refresh_allwin_active();

  //g3d_end_movie();

  return FALSE;
}


/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of an object in the environment
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/


int psp_srch_3D_model_pt_obj(p3d_rob* r, p3d_obj* object, int numpoints, int OnSurface, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point,objCenter;
  double refObjAngle;
  double x,y,z,xo,yo,zo, zcenter=0.0;
  configPt qcurr, qaux;
  int i;
  psp_lst_vertex lstvert2;
  

  p3d_localpath *path=NULL;
  int ntest;



  if (r ==NULL || object == NULL)
    {
      printf("Object or robot not specified \n");
      return FALSE; 

    }

  p3d_get_object_center(object, objCenter);
  if (OnSurface)
    zcenter =((object->BB.zmax - object->BB.zmin)/2) + object->BB.zmin;

  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r); 

  point[3] = 1.0;

  //approach to the object
  
  //if it can reach some points
  
  //next reachable point to reach the other points in the sphere
  
  //psp_gen_ordered_point_list_obj (object, r, numpoints, &lstvert2);    
  psp_gen_ordered_spheric_point_list (object, r, numpoints, &lstvert, PSP_BTSET);    

  printf("Number of vertex %i\n",lstvert.nv);

  for (i=0;i<lstvert.nv;i++)
    {

      if(psp_get_next_ordered_point(point, &lstvert))
	{

	  printf("Point generated %i - %f,%f,%f\n",i,point[0],point[1],point[2]);
      
	  // passing from local to global coords
	  
	  x = point[0]+objCenter[0];
	  y = point[1]+objCenter[1];
	  z = point[2]+objCenter[2]+zcenter;
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////	  
	    xo = objCenter[0];
	    yo = objCenter[1];
	    zo = objCenter[2]+zcenter;
	    p3d_set_and_update_this_robot_conf(r,qcurr);
	    g3d_refresh_allwin_active();
	    if(psp_look_in_two_times_at(r,x,y,z,xo,yo,zo,&qaux))
	      {
		if( (path = p3d_local_planner(r, r->ROBOT_POS,qaux)) ){ 
		  if(!p3d_unvalid_localpath_test(r, path, &ntest)){
		    if (pso_watch2_obj()>=50.0)// can the robot see the object?
		    // if (pso_see_obj())
		      {
			destroy_list_localpath(r,path);
			p3d_copy_config_into(r,qaux, &(r->ROBOT_GOTO));
			return TRUE;
		      }
		  }
		  else
		    printf("Not valid Path\n");
		}
		destroy_list_localpath(r, path);
	      }
	    else
	      printf("Can't place there\n");
	}
      else
	printf("Point %i out of range\n",i);
    }

  /*  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
      p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(r,qcurr);	*/

    
  printf("---- ERROR: Not modeling point found -----\n");
  
  p3d_destroy_config(r,qaux);
  p3d_destroy_config(r,qcurr);

  g3d_refresh_allwin_active();
   
  //g3d_end_movie();

  return FALSE;
}


/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of an object in the environment
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/

int psp_goto_look_obj(p3d_rob* r, p3d_obj* object, int numpoints1, int numpoints2, int OnSurface, hri_bitmapset* PSP_BTSET)
{

  int i=0;
  double tmpMax, tmpMin;
  configPt qcurr;

  qcurr    =  p3d_get_robot_config(r);

  tmpMax   = object->max_pos_range;
  tmpMin   = object->min_pos_range;
  
  object->max_pos_range += .6;
  object->min_pos_range += .6;
  p3d_set_rob_cam_parameters(r,.0,-.10,.0,3.0,7.0,0.75,1.05,10,2,.0,.0);
  while(psp_srch_model_pt_obj(r,object,numpoints1,70.0, PSP_BTSET) && i<numpoints1)
    {
      object->max_pos_range = tmpMax;
      object->min_pos_range = tmpMin;
      printf("trying point %i\n",i);
      p3d_set_rob_cam_parameters(r,-0.20,.0,-0.10,3.0,7.0,0.7,1.05,9,0,.5,.0);

      p3d_set_and_update_this_robot_conf(r,r->ROBOT_GOTO);

      g3d_refresh_allwin_active();

      if  (psp_srch_3D_model_pt_obj(r,object,numpoints2,OnSurface, PSP_BTSET))
	{
	  return TRUE;
	}
      i++;
      p3d_set_rob_cam_parameters(r,.0,.0,.0,3.0,7.0,0.75,1.05,10,2,.0,.0);
    }  
  printf("---- ERROR: Not modeling point found and all possible configurations were tested -----\n"); 
  p3d_set_and_update_this_robot_conf(r,qcurr);
  p3d_destroy_config(r,qcurr);
  return FALSE;
}

/****************************************************************/
/*!
 * \brief Finds a robot configuration in the model area of a search ball in the environment
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/


int psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r, int numpoints, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point;
  double refObjAngle;
  double x,y,xo,yo,zo;
  configPt qcurr, qaux;
  int i;

 
  
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r); 

  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  


  psp_gen_ordered_point_list_searchball (sball, r, numpoints,&lstvert, PSP_BTSET);    

  for (i=0;i<numpoints;i++)
    {

      if(psp_get_next_ordered_point(point, &lstvert))
	{

	  printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      
	  // passing from local to global coords
	  

	  x = point[0]+sball->position[0];
	  y = point[1]+sball->position[1];
	  
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////	  
	    qaux[ROBOTq_X] = x;
	    qaux[ROBOTq_Y] = y;

	    xo = sball->position[0];
	    yo = sball->position[1];
	    zo = sball->position[2];
	    //printf("x -> %f , %f\n",x,xo);
	    //printf("y -> %f , %f\n",y,yo);

     
	    ///Horizontal robot angle
	      refObjAngle = rad_angleOf(x,y,xo,yo);

	      //printf("Angle  %f", refObjAngle);
	      qaux[ROBOTq_RZ] = refObjAngle;

	      if(psp_test_qs(r, qaux, qcurr, sball->position,80.0,1, PSP_BTSET))
		return TRUE;
	}
      else
	printf("Point %i out of range\n",i);
    }

  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(r,qcurr);	
  printf("---- ERROR: Not modeling point found -----\n");
  

  //g3d_end_movie();

  return FALSE;
}

/****************************************************************/
/*!
 * \brief Finds a random robot configuration in the model area of a human camera oriented to the objectif
 * 
 * \param r - robot
 * \param objRob - the human 
 * \param numpoints - number of points to find
 * !

 */
/****************************************************************/



int psp_rnd_srch_model_pt(p3d_rob* r, p3d_rob* objRob, int numpoints, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point,v_aux;
  p3d_jnt *jntPt = objRob->joints[1];
  p3d_jnt *jntPt2;
  //p3d_jnt *jntPtCam = r->joints[r->cam_body_index];
  int i_DoF;
  double f_min, f_max,refHumAngle, lastAngle, lastAngleCam;
  double xMax,xMin,yMax,yMin,x,y,xo,yo,zo;
  //double supVLim, infVLim, centerV;
  configPt qcurr, qaux,objqcurr;
  int i;
 //int res;



  srand((unsigned)time(0)); 
 
  
  xMax =  objRob->max_pos_range;
  xMin =  objRob->min_pos_range * cos(objRob->angle_range/2.0);
  yMax =  objRob->max_pos_range * sin(objRob->angle_range/2.0);
  yMin =  objRob->max_pos_range * sin(-objRob->angle_range/2.0);

  
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r); 
  objqcurr =  p3d_get_robot_config(objRob);
  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  
  v_aux[0] = 0.0;
  v_aux[1] = 0.0;

  lastAngle =  rad_angleOf(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);//qaux[ROBOTq_RZ]; //get_robot_angle_rad(r);
  if (lastAngle>2*M_PI)
    lastAngle -=2*M_PI;
  lastAngleCam  = qaux[ROBOTq_PAN];
 
  jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_PAN,&i_DoF);
  p3d_jnt_get_dof_bounds(jntPt2,i_DoF, &f_min, &f_max);

  for (i=0;i<numpoints;i++)
    {

      //Random point type
     psp_gen_rand_point(point,xMax,xMin,yMax,yMin);


      printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      if (p3d_is_in_pos_area(objRob,point[0],point[1],FALSE))
	{
	  // passing from local to global coords
	  p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);


	  x = v_aux[0];
	  y = v_aux[1];
	      
	  
	  qaux[ROBOTq_X] = x;
	  qaux[ROBOTq_Y] = y;

	 // qaux[ROBOTq_RZ]   = lastAngle ;
	  printf("Original  %f,%f \n",qaux[ROBOTq_RZ],qaux[ROBOTq_PAN]);
	  
	  //qaux[ROBOTq_RZ] = rad_angleOf(x,y,objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);
	  
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////
	    p3d_get_robot_center(objRob, point);
	    //p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);g3d_set_win_drawer(G3D_WIN, g3d_draw_trace); g3d_set_win_drawer(G3D_WIN, g3d_draw);
	    xo = point[0];
	    yo = point[1];
	    zo = point[2];
	    printf("x -> %f , %f\n",xo, objqcurr[HUMANq_X]);
	    printf("y -> %f , %f\n",yo, objqcurr[HUMANq_Y]);
	    printf("z -> %f , %f\n",zo, objqcurr[HUMANq_Y+1]);
	    ///Horizontal robot angle
	    refHumAngle = rad_angleOf(x,y,xo,yo);

	    printf("Angle  %f max %f\n", refHumAngle,f_max);
	    qaux[ROBOTq_RZ] = refHumAngle;

	    /* This part is for jido, just for looking at the objectif with the body in a perpendicular direction from the robots's
	       avoiding auto oclusions with de arm
	      if (refHumAngle<M_PI)
	      {
		//Fis
		if (refHumAngle>= f_max)
		  {
		    qaux[ROBOTq_RZ] = refHumAngle - f_max;
		    qaux[ROBOTq_PAN] = f_max; 
		   
		  }
		else
		  {
		    //qaux[ROBOTq_RZ+1] = refHumAngle;
		    
		    qaux[ROBOTq_RZ] = -(f_max - refHumAngle) ;
		    qaux[ROBOTq_PAN] = f_max; 
		    
		  }
	      }
	    else
	      {
		if (refHumAngle<= 2*M_PI+f_min)
		  {
		    // Third cuadrant              
		    qaux[ROBOTq_RZ]    = (refHumAngle - 2*M_PI)  - f_min;
		    qaux[ROBOTq_PAN]  = f_min; 
		    
		  }
		else
		  {
		    // Fourth cuadrant
		    qaux[ROBOTq_RZ] = -(f_min - (refHumAngle - 2*M_PI));
		    qaux[ROBOTq_PAN]  = f_min;
		   
		  }
	      }
	    */

	    if(psp_test_qs(r, qaux, qcurr,point,80.0,1, PSP_BTSET))
		return TRUE;
	}
      else
	{
	  //i--;
	  printf("Not in range point %i - %f,%f\n",i,point[0],point[1]);
	}
    }

  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(r,qcurr);	
  printf("---- ERROR: Not modeling point found -----\n");
  

  //g3d_end_movie();

  return FALSE;
}




/**********************************************************************/

// Perspective reasonning  functions


/**********************************************************************/


/****************************************************************/
/*!
 * \brief Generates a list of observed objects
 * 
 * !
 */
/****************************************************************/

void psr_get_obj_list()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_obj *o;
  p3d_rob *r, *human;

  int no = envPt->no;
  int i;

  printf("------------ Robots: %i\n", envPt->nr);

  p3d_deselect_all_objects();

  for(i=0;i<envPt->nr;i++)
    {
      p3d_select_robot_to_view(envPt->robot[i]);
      
      if(pso_look_obj())
	{
	  printf("Robot observed: %s\n",  envPt->robot[i]->name);
	}
      p3d_deselect_robot_to_view(envPt->robot[i]);
      if (strstr(envPt->robot[i]->name,"human"))
	{
	  human = envPt->robot[i];
	}
      else
	if(!strcmp("robot",envPt->robot[i]->name))
	  {
	    r = envPt->robot[i];
	  }
     // envPt->robot[i]->caption_selected = FALSE;
    }


  printf("------------ Objects: %i\n", no);

  p3d_deselect_all_objects();

  //search for obstacles in the environment
  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
     // found = FALSE;
      
      //p3d_select_object_to_view(&o);
      o->caption_selected = 1;
      
      if(pso_look_obj())
	{
	  printf("Object observed: %s\n", o->name);
	}
//      p3d_unselect_object_to_view(&o);
      o->caption_selected = 0;

    }

}


/****************************************************************/
/*!
 * \brief Finds a human configuration to look to a point
 * 
 * \param r - robot
 * \param xo, yo, zo - point
 * \param resq - resulting q
 * !
 */
/****************************************************************/

static int psr_human_look_at(p3d_rob* human, double x, double y, double z)
{
  configPt HumConf;
  p3d_vector4 pointHead;//, pointVw, pointView;
  double anglexy, anglexz;
  int HumanHead1, HumanHead2;

  p3d_get_object_center(human->o[human->cam_body_index], pointHead);
  
/*   pointVw[0] = 1.0; */
/*   pointVw[1] = 0.0; */
/*   pointVw[2] = 0.0; */
/*   pointVw[0] = 1.0; */

/*   p3d_matvec4Mult(human->o[human->cam_body_index].jnt.abs_pos,pointVw,pointView); */

  //anglexy = rad_angleOf(0.0, 0.0, x-pointHead[0], y-pointHead[1]);
  //anglexz = rad_angleOf(0.0, 0.0, x-pointHead[0], z-pointHead[2]);

  //  printf("Coordinates Angles: %f %f  \n",anglexy,anglexz);

  p3d_psp_cartesian2spherical( x-pointHead[0],  y-pointHead[1],  z-pointHead[2], .0, .0, .0, &anglexy, &anglexz);
  
  anglexz-=M_PI/2;
  printf("Coordinates Spherical Angles: %f %f  \n",anglexy,anglexz);
  
  HumConf    = p3d_get_robot_config(human);

  anglexy -= HumConf[ROBOTq_RZ];
  HumanHead1 = human->joints[54]->index_dof;
  HumanHead2 = human->joints[55]->index_dof;

  HumConf[HumanHead1] = anglexy;
  HumConf[HumanHead2] = anglexz;

  p3d_set_and_update_this_robot_conf(human, HumConf);	
  
  return(0);
}


/****************************************************************/
/*!
 * \brief Gives objects that are pointed by human left hand
 * 
 * \param r - interacting robot
 * \param human - human 
 * \param 
 * !
 */
/****************************************************************/

void psr_get_human_left_pointing(p3d_rob* human, p3d_rob* r, hri_bitmapset* PSP_BTSET)
{
  p3d_jnt *jntPt = human->joints[27];//human->o[6]->jnt;
  psr_get_pointing_from_joint(r,jntPt,5, PSP_BTSET);

}

void psr_get_human_pointing_from_joint_number(p3d_rob* human, p3d_rob* r, int jntIdx, hri_bitmapset* PSP_BTSET)
{
  int i,j;
  p3d_jnt *jntPt = human->joints[jntIdx];//human->o[6]->jnt; //56 or 57 for balls
  psr_get_pointing_from_joint(r,jntPt,1, PSP_BTSET);
  for(i=0 ; i<=3 ; i++)
    {
      for(j=0 ; j<=3 ; j++)
	printf("%f ", jntPt->abs_pos[i][j]);
      printf("---\n");
    }
  
}

void psr_get_pointing_from_joint(p3d_rob* r,  p3d_jnt *jntPt, int frameType, hri_bitmapset* PSP_BTSET)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_obj *o = NULL, *oSel = NULL;
  psp_cone cone;
  p3d_vector4 pointHand, pointBase, pointObject;
  p3d_vector4 conebase;
  double disttocenter, mindist, maxdist=0.0;
  double pref=0.0, prefAcum=0.0;
  int i, found=0, found2=0;

  conebase[0] = 0.0;
  conebase[1] = 0.0;
  conebase[2] = 0.0;
  conebase[3] = 1.0;

  p3d_matvec4Mult(jntPt->abs_pos,conebase,pointHand);

  cone.origin[0] = pointHand[0];
  cone.origin[1] = pointHand[1];
  cone.origin[2] = pointHand[2];
  cone.h        = 4.0;
  cone.angle    = 0.5;
  mindist       = cone.h;

  switch (frameType)
    {
    case 0:// X
      conebase[0] = cone.h;//cone.h; 
      break;
    case 1:// Y
      conebase[1] = cone.h;
      break;
    case 2:// Z
      conebase[2] = cone.h;
      break;  
// same 3 but on negative frame    
    case 3:// -X
      conebase[0] = -cone.h;//cone.h; 
      break;
    case 4:// -Y
      conebase[1] = -cone.h;
      break;
    case 5:// -Z
      conebase[2] = -cone.h;
      break; 
    }

  conebase[3] = 1.0;

//  for(i=0 ; i<=3 ; i++)
 //   for(j=0 ; j<=3 ; j++)
  //    matrix[i][j] = jntPt->abs_pos[i][j];
  ox = oy = oz =0.0;

  p3d_matvec4Mult(jntPt->abs_pos,conebase,pointBase);

 
  for(i=0;i<envPt->no;i++)
  {
      o = envPt->o[i];
      p3d_get_object_center(o, pointObject);     
     // printf("Obstacle %s center point: %f %f %f \n", o->name,pointObject[0],pointObject[1],pointObject[2]);
      if (p3d_psp_is_point_in_a_cone(pointObject, pointHand, pointBase, cone.angle, &disttocenter))
	  {

		  pref = psr_preference_obj(pointObject, disttocenter, pointHand);
		  //printf("Obstacle in cone: %s preference %f\n", o->name,pref);
		  if (!found2)
		  {
			  ox += pointObject[0] * pref;
			  oy += pointObject[1] * pref;
			  oz += pointObject[2] * pref;
		  }
		  else
		  {
			  ox = pointObject[0] * pref;
			  oy = pointObject[1] * pref;
			  oz = pointObject[2] * pref;
			  found2=0;
		  }
		  found++;
		  prefAcum += pref; 
		  if (disttocenter>maxdist)
		  {
			  oSel=o;
			  maxdist = disttocenter;
		  }
	  }
      else
	  {
	//	  printf("-- NOT in cone but at distance: %f\n", disttocenter);
		  
		  if (disttocenter<mindist && disttocenter<cone.h/2.0 && !found)
		  {
			  ox = pointObject[0];
			  oy = pointObject[1];
			  oz = pointObject[2];
			  mindist = disttocenter;
			  //printf("is minimal--\n");
			  oSel=o;
			  found2=1;
		  }
		  
	  }
  }
  if (found>1)
  { 
      srchball.position[0] = ox = ox / prefAcum;
      srchball.position[1] = oy = oy / prefAcum;
      srchball.position[2] = oz = oz / prefAcum;
      sphereActive = 1;  
	  
      srchball.distMin = maxdist + p3d_get_obj_centertoborder_distance(oSel);
      srchball.distMax =  srchball.distMin + (p3d_get_robot_centertoborder_distance(r)*2);
	  
      if (psp_srch_model_pt_searchball(&srchball,r, 60, PSP_BTSET))
	  {
	    printf("search ball found\n");
	  }
	  
  }
  else
    if (found || found2)
      {
	//printf("Entering\n");
	sphereActive = 1;  
	if (found2)
	  {
	    srchball.position[0] = ox;
	    srchball.position[1] = oy;
	    srchball.position[2] = oz;
	  }
	else
	  {
	    srchball.position[0] = ox = ox / prefAcum;
	    srchball.position[1] = oy = oy / prefAcum;
	    srchball.position[2] = oz = oz / prefAcum;
	  }

	p3d_get_object_center(oSel, pointObject);
	oSel->caption_selected = 1;
	oSel->min_pos_range =  p3d_get_obj_centertoborder_distance(oSel);
	oSel->max_pos_range =  oSel->min_pos_range + (p3d_get_robot_centertoborder_distance(r)*2);
	printf("Obstacle Selected: %s at coords %f, %f, %f \n", oSel->name,ox,oy,oz);
	psp_srch_model_pt_obj(r,oSel, 60,80.0, PSP_BTSET);
      }

    //psr_human_look_at(human,ox,oy,oz);


  lx1 = pointHand[0];
  ly1 = pointHand[1];
  lz1 = pointHand[2];
  lx2 = pointBase[0];
  ly2 = pointBase[1];
  lz2 = pointBase[2];
  lrad = cone.h * sin(cone.angle/2.0);
  //lrady = 
  printf("end searching -----\n");
  
  //printf("%s point : %f %f %f \n",human->o[6]->name,pointHand[0],pointHand[1],pointHand[2]);

}



/**********************************************************************

configPt p3d_psp_srch_lookatpoint(p3d_rob* r, int numpoints)
{
  p3d_vector4 point,v_aux;
  int i;
 //int res;
  
  
  srand((unsigned)time(0)); 
  
  
  xMax =  objRob->max_pos_range;
  xMin =  objRob->min_pos_range * cos(objRob->angle_range/2.0);
  yMax =  objRob->max_pos_range * sin(objRob->angle_range/2.0);
  yMin =  objRob->max_pos_range * sin(-objRob->angle_range/2.0);
  
  
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r); 
  objqcurr =  p3d_get_robot_config(objRob);
  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  
  v_aux[0] = 0.0;
  v_aux[1] = 0.0;

  lastAngle =  rad_angleOf(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);//qaux[ROBOTq_RZ]; //get_robot_angle_rad(r);
  if (lastAngle>M_PI)
    lastAngle -=2*M_PI;
  lastAngleCam  = qaux[ROBOTq_PAN];
 
  jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_PAN,&i_DoF);
  p3d_jnt_get_dof_bounds(jntPt2,i_DoF, &f_min, &f_max);


  for (i=0;i<numpoints;i++)
    {
      p3d_psp_gen_rand_point(point,xMax,xMin,yMax,yMin);
      printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      if (p3d_is_in_pos_area(objRob,point[0],point[1]))
	{
	  // passing from local to global coords
	  p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);
	  
	  
	  x = v_aux[0];
	  y = v_aux[1];
	  
	  
	  qaux[ROBOTq_X] = x;
	  qaux[ROBOTq_Y] = y;
	  
	  qaux[ROBOTq_RZ]   = lastAngle ;
	  printf("Original  %f,%f \n",qaux[ROBOTq_RZ],qaux[ROBOTq_PAN]);
	  
	  //qaux[ROBOTq_RZ] = rad_angleOf(x,y,objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);
	  
	  ///////////
	    /// Modifying Robot and its camera angle
	    ///////////
	    p3d_get_robot_center(objRob, point);
	    //p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);g3d_set_win_drawer(G3D_WIN, g3d_draw_trace); g3d_set_win_drawer(G3D_WIN, g3d_draw);
	    xo = point[0];
	    yo = point[1];
	    zo = point[2];
	    printf("x -> %f , %f\n",xo, objqcurr[HUMANq_X]);
	    printf("y -> %f , %f\n",yo, objqcurr[HUMANq_Y]);
	    printf("z -> %f , %f\n",zo, objqcurr[HUMANq_Y+1]);
	    ///Horizontal robot angle
	      refHumAngle = rad_angleOf(x,y,xo,yo);
	      printf("Angle  %f max %f\n", refHumAngle,f_max);
	      //qaux[ROBOTq_RZ] = refHumAngle;
	      
	      
	      
	      if((qend=(configPt)psp_test_qs(r, qaux, qcurr,point))!=NULL)
		return qend;
	      
	}
      else
	{
	  i--;
	  printf("Not in range point %i - %f,%f\n",i,point[0],point[1]);
	}
    }
  
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
  p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(r,qcurr);	
  printf("---- ERROR: Not modeling point found -----\n");
  
  
  //g3d_end_movie();
  
  return NULL;
}

*/

/**************************************************************************************************/
/****************************************/
/* Percentage Observation Functions     */
/****************************************/

/***************************************
watch = gives object observation %
****************************************/
static double pso_watch2_obj()
{ 

  int        w=0,h=0; 
  G3D_Window *win = g3d_get_win_by_name("Perspective");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);
  fl_get_winsize(FL_ObjWin(ob),&w,&h);

  int        i, greenCount=0, totalCount=0; 
  double total=0.0;


  glDrawBuffer (GL_BACK);//draw window function makes swap coping back to front
  glReadBuffer(GL_BACK) ; 
  GLfloat* pixels = MY_ALLOC(GLfloat,(w*h*3));

  for (i=0;i<(h*w*3);i+=3)
    {
      pixels[i]=0.0;
    }

  glLoadIdentity();
  g3d_set_win_draw_mode(win,OBJECTIF);
  
  g3d_refresh_win(win);
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT, pixels);

  for (i=0;i<(h*w*3);i+=3)
    {
      if (pixels[i]>=0.0 && pixels[i]!=1.0)
	{
	  if ((pixels[i+1]>0.7) &&  (pixels[i+2]==0.0))
	    { 
	      //Green pixels
	      totalCount++;
              //printf("1");
	    }
	  //else
	  // printf("Blue\n");
	  //printf("Pixel %i -> %f,%f,%f\n",i,pixels[i],pixels[i+1],pixels[i+2]);
        
	}
      //else
      //  printf(".");
      //if(i/w==1)
      //  printf("_ \n");
       
          
    }
 
  glLoadIdentity();  
  g3d_set_win_draw_mode(win,DIFFERENCE);
  g3d_refresh_win(win); 
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,pixels);
  //decodificar cada pixel
  for (i=0;i<(h*w*3);i+=3)
    {
      if (pixels[i]>=0.0 && pixels[i]!=1.0)
	{
	  if ((pixels[i+1]>0.7) &&  (pixels[i+2]==0.0))
	    { 
	      //Green pixels
	      greenCount++;
	    }

	}
    }  
  
  if (totalCount>0)
    total = (greenCount*100.0)/(totalCount*1.0);
  else

    total = 0.0;

  MY_FREE(pixels,GLfloat,w*h*3);
  g3d_set_win_draw_mode(win,NORMAL);
  g3d_refresh_win(win); 
  return total;
}

/*///////////////////////////////////////////////////////*/

static double pso_watch_obj()
{ 
  //GLubyte pPixels[4];

  int        w=0,h=0; 
  G3D_Window *win = g3d_get_win_by_name("Perspective");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);
  FL_FORM *tmpwin = NULL;
  fl_get_winsize(FL_ObjWin(ob),&w,&h);

  // float* pixels=malloc(sizeof(GL_FLOAT)*w*h*3);
  int        i, greenCount=0, totalCount=0; 
  double total=0.0;


  tmpwin= ((FL_FORM *)win->form);
  fl_raise_form(tmpwin);

 
  //AKIN FIX if(glXGetCurrentContext() !=  (void*)fl_get_glcanvas_context(ob))
    { 
      printf("Raising window -> %s\n",win->name);
      //AKIN FIX  glXMakeCurrent(fl_display,FL_ObjWin(ob), (__GLXcontextRec*)fl_get_glcanvas_context(ob));
    }


  if (w<=0 || h<=0 || !ob)
    {
      printf("Canvas Size Problem -> %i,%i\n",w,h);
      return 0.0;
    }

  GLfloat* pixels = MY_ALLOC(GLfloat,(w*h*3));

  for (i=0;i<(h*w*3);i+=3)
    {
      pixels[i]=0.0;
    }
  //glDisable(GL_LIGHTING);
  //glDisable(GL_LIGHT0);
  glXWaitGL();
  glLoadIdentity();
  g3d_set_win_draw_mode(win,OBJECTIF);
  //refresh
  //g3d_draw_win(win);  //g3d_draw_win(win); 
  g3d_refresh_win(win);
  //glXWaitGL();
  //g3d_refresh_allwin_active();
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT, pixels);
  glXWaitGL();
  //glFlush();
  //decodificar cada pixel
  for (i=0;i<(h*w*3);i+=3)
    {
      if (pixels[i]>=0.0 && pixels[i]!=1.0)
	{
	  if ((pixels[i+1]>0.7) &&  (pixels[i+2]==0.0))
	    { 
	      //Green pixels
	      totalCount++;
              //printf("1");
	    }
	  //else
	  // printf("Blue\n");
	  //printf("Pixel %i -> %f,%f,%f\n",i,pixels[i],pixels[i+1],pixels[i+2]);
        
	}
      //else
      //  printf(".");
      //if(i/w==1)
      //  printf("_ \n");
       
          
    }
 
  glLoadIdentity();  
  g3d_set_win_draw_mode(win,DIFFERENCE);
  //refresh
  //glFlush();
  g3d_refresh_win(win); 
  //glXWaitGL();
  // g3d_refresh_allwin_active();
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,pixels);
  glXWaitGL();
  //glFlush();
  //decodificar cada pixel
  for (i=0;i<(h*w*3);i+=3)
    {
      if (pixels[i]>=0.0 && pixels[i]!=1.0)
	{
	  if ((pixels[i+1]>0.7) &&  (pixels[i+2]==0.0))
	    { 
	      //Green pixels
	      greenCount++;
	    }

	}
    }  
  
  //glEnable(GL_LIGHTING);
  //glEnable(GL_LIGHT0);
  //g3d_set_win_draw_mode(win,NORMAL);
  //g3d_draw_win(win); 
  //glXSwapBuffers(fl_display,fl_get_canvas_id(ob));
  if (totalCount>0)
    total = (greenCount*100.0)/(totalCount*1.0);
  else
    total = 0.0;

  //printf("Watch %g %i %i \n",total,greenCount,totalCount);
  MY_FREE(pixels,GLfloat,w*h*3);
  //free(pixels);
  //free(apixels);
  g3d_set_win_draw_mode(win,NORMAL);
  g3d_refresh_win(win); 
  return total;
}
/**********************************/
/* Bool Observation Functions     */
/**********************************/

/***************************************
See = if object observation a 100% (>95%)
****************************************/

static int pso_see_obj()
{
  if (pso_watch2_obj()>=90.0)
    return 1;
  return 0;
}


/***************************************
look = if object observation > 30%
****************************************/
static int pso_look_obj()
{
  if (pso_watch_obj()>=15.0)
    return 1;
  return 0;
}

/**************************************
Perceive = if object in the percetion area
****************************************/
static double pso_perceive_obj()
{
  return 0.0;
}


/**********************************************************************/

/****************************************/
/* Initialization functions             */
/****************************************/

static void psp_gen_confs(int n)
{
int i;
  for (i=0; i<n; i++)
    {
      theqs[i] = p3d_get_robot_config(PSP_ROBOT);
    }
}

int p3d_init_robot_parameters()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *currobotPt;
  int i;
  for(i=0; i<envPt->nr; i++){
    currobotPt=envPt->robot[i];
    if(strstr(currobotPt->name,"human")) //for all humans
      {
        
	p3d_set_rob_cam_parameters(currobotPt,.1,.0,.0,3.0,7.0,1.0,2.0,15,2,.0,.0);
	currobotPt->angle_range   = 2.0;
	currobotPt->max_pos_range = 1.3; //3.0;	
	currobotPt->min_pos_range = 1.2; //2.0;

      }
    else
      if(strstr("robot",currobotPt->name))
	{

	  #ifdef JIDO 
	  p3d_set_rob_cam_parameters(currobotPt,.0,-.10,.0,3.0,7.0,0.75,1.05,10,2,.0,.0);
	  #endif
	  #ifdef BH
	  p3d_set_rob_cam_parameters(currobotPt,.05,-.05,.0,3.0,7.0,0.75,1.05,4,2,.0,.0);
	  #endif

	  currobotPt->angle_range   = 1.0;
	  currobotPt->max_pos_range = 3.0;
	  currobotPt->min_pos_range = 1.0;
	  PSP_ROBOT = currobotPt;
	}	       
    //printf("Robot %i %s\n",i,envPt->robot[i]->name);

  }
  PSP_GIK = hri_gik_create_gik();
  PSP_GIK2 = hri_gik_create_gik();
  PSP_GIK3 = hri_gik_create_gik();
  p3d_init_object_parameters_by_name("table1",0.5,1.0);
  PSP_DEACTIVATE_AUTOHIDE =0;
  psp_gen_confs(40);
  return 0;
}

int p3d_init_object_parameters_by_name(char *objName,  double min, double max)
{

  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no,i, cont=0;
  pp3d_obj o;

 
  no = envPt->no;

  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      if (strcmp(o->name,objName)==0)
	{
	  o->max_pos_range = max;
	  o->min_pos_range = min;
	  cont++;
	}
    }
  if (cont>0)
    return cont;
 
  return 0;

}

int p3d_init_all_object_parameters_by_type(char *objType,  double min, double max)
{

  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no,i, cont=0;
  pp3d_obj o;
 
  no = envPt->no;

  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      if (strstr(o->name,objType))
	{
	  o->max_pos_range = max;
	  o->min_pos_range = min;
	  cont++;
	}
    }
  if (cont>0)
    return cont;
 
  return 0;

}

void psp_search_for_objectives(p3d_rob *robot, p3d_vector3 point)
{
  
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int found, contr=0, conto=0;
  int nr, no, i, nrs=0;
  int *ids;
  p3d_vector3 centerP1={0.0};
  p3d_vector4 centerP2={0.0};
  pp3d_obj o;

  nr = envPt->nr;

  for(i=0;i<nr;i++)
    {

      if(p3d_get_rob_select_status(envPt->robot[i]))
	{
	  ids[nrs]= envPt->robot[i]->num;
	  nrs++;
	  //get centerp
	  p3d_get_robot_center(envPt->robot[i], centerP2);
	  centerP1[0] += centerP2[0];
	  centerP1[1] += centerP2[1];
	  centerP1[2] += centerP2[2];
	  contr++;
	  //search for max_pos_range and  min_pos_range
	}

    }
  no = envPt->no;

  //search for obstacles in the environment that aren't in seleted robot(s)
  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      found = FALSE;
      if(o->caption_selected)
	{
	  if (o->is_used_in_device_flag)
	    {
	      //	      for (idx=0;i<nrs;idx++)
	      if (p3d_get_rob_select_status(o->jnt->rob))// &&  (o->jnt->rob->num == ids[idx]))
		{
		  found = TRUE;
		}
	    }
	  if (!found)
	    {
	      //get centerp
	      p3d_get_object_center(o,centerP2);
	      centerP1[0] += centerP2[0];
	      centerP1[1] += centerP2[1];
	      centerP1[2] += centerP2[2];
	      conto++;
	      //modify

	    }

	}
    }

  point[0] = centerP1[0] / (contr+conto);
  point[1] = centerP1[1] / (contr+conto);
  point[2] = centerP1[2] / (contr+conto);

}


int psp_srch_for_target_obj(p3d_rob *robot, int numPoints, int searchMode, hri_bitmapset* PSP_BTSET )
{
  
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int found;
  int no, i;
  pp3d_obj o;

  no = envPt->no;
  found = FALSE;
  //search for obstacles in the environment that aren't in seleted robot(s)
  for(i=0;i<no && !found;i++)
    {
      o = envPt->o[i];
      found = FALSE;
      if(o->caption_selected)
	{
	  found=TRUE;
	}
    }
  if (found)
    {
      if (searchMode < 2)
	{
	  if (searchMode > 0)
	    PSP_DEACTIVATE_AUTOHIDE = 1;
	  if (psp_srch_model_pt_obj(robot,o,numPoints,80.0, PSP_BTSET))
	    {
	       PSP_DEACTIVATE_AUTOHIDE = 0;
	       return TRUE;
	    }
	   PSP_DEACTIVATE_AUTOHIDE = 0;
	}
      else
	if ( psp_goto_look_obj(robot,o,40,20,1,PSP_BTSET))
	  return TRUE;

    }
  printf("NO selected object found\n");
  return FALSE;
}



void psp_add_element(psp_lst_elements *lstel, psp_obs_element *elem)
{



}


/**********************************************************************/


/****************************************/
/* Select and unselect Robot functions */
/****************************************/

void p3d_select_robot_to_view(p3d_rob *robotPt)
{

  int nb,i;
  pp3d_obj o;

 
  nb = robotPt->no; //number of objects of the robot
  robotPt->caption_selected = 1;

  for(i=0;i<nb;i++)
    {
      o = robotPt->o[i];
      o->caption_selected = 1;
    }

}

/***/

void p3d_deselect_robot_to_view(p3d_rob *robotPt)
{

  int nb,i;
  pp3d_obj o;

 
  nb = robotPt->no;
  robotPt->caption_selected = 0;

  for(i=0;i<nb;i++)
    {
      o = robotPt->o[i];
      o->caption_selected = 0;
    }

}

int p3d_get_rob_select_status(p3d_rob *robotPt)
{
  return robotPt->caption_selected;
}

//incomplet

void p3d_deselect_all_objects()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no,i;
  pp3d_obj o;

 
  no = envPt->no;
  //robotPt->caption_selected = 0;

  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      o->caption_selected = 0;
    }

}

/****************************************/
/* Select and unselect object functions */
/****************************************/

void p3d_select_object_to_view(p3d_obj *objectPt)
{
  objectPt->caption_selected = 1;
}


int psp_select_object_to_view_by_name(char *objName)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no,i, cont=0;
  pp3d_obj o;

 
  no = envPt->no;

  for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      if (strcmp(o->name,objName)==0)
	{
	   o->caption_selected = 1;
	   cont++;
	}
    }
  if (cont>0)
    {
      printf("Object %s found %i time(s)\n",objName,cont);
      return TRUE;
    }
  else
    {
      printf("Object %s NOT found \n",objName);
      return FALSE;  
    } 

}


void p3d_unselect_object_to_view(p3d_obj *objectPt)
{
  objectPt->caption_selected = 0;
}

int p3d_get_obj_select_status(p3d_obj *objectPt)
{
  return objectPt->caption_selected;
}


void p3d_set_body_selection(p3d_rob *r, int body, int val )
{
  r->o[body]->caption_selected = val;
}


/****************************************/
/*     Auxiliar robot Functions         */
/****************************************/

//void p3d_copy_robot()

void g3d_psp_draw_lookatpoint(p3d_rob *robot)
{
  psp_obs_vertex *srchbPt = (psp_obs_vertex*) robot->lookatpoint;
  if (srchbPt)
    {
      glPushMatrix();
      g3d_drawSphere(srchbPt->pos[0],srchbPt->pos[1],srchbPt->pos[2], 0.2, Green,NULL);
      glPopMatrix();
    }
}


/*


void p3d_psp_set_search_ball_pos(p3d_rob *robot, double x, double y, double z)
{
  robot->searchBall->position[0] = x;
  robot->searchBall->position[1] = y;
  robot->searchBall->position[2] = z;
}


void p3d_psp_set_search_ball_status(p3d_rob *robot, int status)
{
  robot->searchBall->active = status;
}

*/


static double p3d_get_vertical_center(p3d_rob* rob)
{
  return (((rob->BB.zmax - rob->BB.zmin)/2) + rob->BB.zmin);
}


//Center point based on BB of the robot
void p3d_get_robot_center(p3d_rob* rob, p3d_vector4 pointc)
{
	pointc[0] = (((rob->BB.xmax - rob->BB.xmin)/2) + rob->BB.xmin);
	pointc[1] = (((rob->BB.ymax - rob->BB.ymin)/2) + rob->BB.ymin);
	pointc[2] = (((rob->BB.zmax - rob->BB.zmin)/2) + rob->BB.zmin);
}

//Center point based on BB of the object
void p3d_get_object_center(p3d_obj* obj, p3d_vector4 pointc)
{
	pointc[0] = (((obj->BB.xmax - obj->BB.xmin)/2) + obj->BB.xmin);
	pointc[1] = (((obj->BB.ymax - obj->BB.ymin)/2) + obj->BB.ymin);
	pointc[2] = (((obj->BB.zmax - obj->BB.zmin)/2) + obj->BB.zmin);
}

//Maximum distance between center point and one corner of de bb based on BB of the object
static double p3d_get_obj_centertoborder_distance(p3d_obj* obj)
{

  double distancex, distancey, distancez;
  distancex = (obj->BB.xmax - obj->BB.xmin)/2;
  distancey = (obj->BB.ymax - obj->BB.ymin)/2;
  distancez = (obj->BB.zmax - obj->BB.zmin)/2;

  if (distancex<distancey)
    return distancey+distancex/2;
  else
    return distancex+distancey/2;
  
}

//Maximum distance between center point and one corner of de bb based on BB of the robot
static double p3d_get_robot_centertoborder_distance(p3d_rob *r)
{

  double distancex, distancey, distancez;
  distancex = (r->BB.xmax - r->BB.xmin)/2;
  distancey = (r->BB.ymax - r->BB.ymin)/2;
  distancez = (r->BB.zmax - r->BB.zmin)/2;

  if (distancex<distancey)
    return distancey+distancex/2;
  else
    return distancex+distancey/2;
  
}



/****************************************************************/
/*!
 * \brief Finds the mimimum distance between a point and a line
 * 
 * \param p  point
 * \param l1 a point on the line
 * \param l2 an other point on the line
 * !

 */
/****************************************************************/
double p3d_psp_pointtolinedist(p3d_vector3 p, p3d_vector3 l1, p3d_vector3 l2)
{
  p3d_vector3 l1mp;
  p3d_vector3 l2ml1;
  p3d_vector3 crossprod;
  
  p3d_vectSub(l1,p,l1mp);
  p3d_vectSub(l2,l1,l2ml1);
  p3d_vectXprod(l1mp,l2ml1,crossprod);

  return (p3d_vectNorm(crossprod)/p3d_vectNorm(l2ml1));
}

/****************************************************************/
/*!
 * \brief Converts a cartesian coordinate to a spherical one
 * 
 * \param x,y,z point
 * \param originx,originy,originz origin point 
 * \param phi,theta resulting angles
 * !

 */
/****************************************************************/
void p3d_psp_cartesian2spherical(double x, double y, double z,
				double originx, double originy, double originz,
				double *phi, double *theta)
{
  double distance = DISTANCE3D(x,y,z,originx,originy,originz);
 
  *phi = atan2( (y-originy),(x-originx) );
  *theta = acos( (z-originz)/distance );
  
}

/****************************************************************/
/*!
 * \brief Converts a spherical coordinate to a cartesian one
 * 
 * \param x,y,z     - original point
 * \param rad       - radius 
 * \param phi,theta - angles
 * \param point     - resulting point
 * 
 * !

 */
/****************************************************************/
void p3d_psp_spherical2cartesian(double x, double y, double z, 
				 double rad, double phi, double theta, 
				 p3d_vector4 point)

{

  point[0] = x + rad * cos(theta) * sin(phi);
  point[1] = y + rad * sin(theta) * sin(phi);
  point[2] = z + rad * cos(phi);
  
}


/****************************************************************/
/*!
 * \brief finds if a point is inside a cone or not
 * 
 * \param p         - point to compare
 * \param conep     - original point of the cone
 * \param conep2    - base point of the cone
 * \param coneangle - cone's aperture angle 
 * 
 * !

 */
/****************************************************************/

int p3d_psp_is_point_in_a_cone(p3d_vector4 p, p3d_vector4 conep, p3d_vector4 conep2  , double coneangle, double *distf)
{

  p3d_vector3 paux, conepaux, conepaux2;
  double disttoorigin, distofline;
  paux[0]     = p[0];
  paux[1]     = p[1];
  paux[2]     = p[2];
  conepaux[0] = conep[0];
  conepaux[1] = conep[1];
  conepaux[2] = conep[2];
  conepaux2[0] = conep2[0];
  conepaux2[1] = conep2[1];
  conepaux2[2] = conep2[2];

  disttoorigin =  DISTANCE3D( paux[0], paux[1], paux[2], conep[0],conep[1],conep[2]);
  distofline   =  DISTANCE3D( conep[0],conep[1],conep[2], conep2[0],conep2[1],conep2[2] );

  double a    =  p3d_psp_pointtolinedist(paux,conepaux,conepaux2);
  double c    =  DISTANCE3D(conepaux[0],conepaux[1],conepaux[2],paux[0],paux[1],paux[2]); //distance to base
  double alfa =  asin((a*sin(1.57))/c);
  //printf("a: %f c:%f  alfa:%f \n", a,c,alfa);
  *distf = a;
  if (alfa<coneangle/2.0 && (disttoorigin<=distofline && c<=distofline))
    return 1;
  else
    return 0;

}

static void psp_draw_confs()
{
  int i;
//  G3D_Window* win = g3d_get_main_win();
  printf("cuantos %i\n",qindex);
  if (PSP_ROBOT)
    for (i=0; i<qindex; i++)
      {
//	p3d_set_and_update_this_robot_conf(PSP_ROBOT,theqs[qindex]);
//	g3d_draw_robot(PSP_ROBOT,win);
	printf("passing %i %f,%f\n",i,theqs[qindex][ROBOTq_X],theqs[qindex][ROBOTq_Y]);
	//g3d_refresh_allwin_active(); 
      }

}


void  psp_draw_test()
{
  double *color_vect = NULL; 
  int i;
//  double c    =  DISTANCE3D(lx1, ly1, lz1, lx2, ly2, lz2);
  
 // double lradz = sqrt((c*c)+(lrad*lrad));

   g3d_drawOneLine(lx1, ly1, lz1, lx2, ly2, lz2, Green, color_vect);
  
/*   g3d_drawOneLine(lx1, ly1, lz1, lx2+lrad, lz2 , ly2+lrad,  Red, color_vect); */

/*   g3d_drawOneLine(lx1, ly1, lz1, lx2-lrad, lz2, ly2+lrad,  Red, color_vect); */

/*   g3d_drawOneLine(lx1, ly1, lz1, lx2-lrad, lz2, ly2-lrad,  Red, color_vect); */

/*   g3d_drawOneLine(lx1, ly1, lz1, lx2+lrad, lz2, ly2-lrad,  Red, color_vect); */

  // g3d_drawSphere(lx2, ly2, lz2,lrad,Red, color_vect);

  if (sphereActive)
    {
      g3d_drawSphere(ox, oy, oz,0.1,Blue, color_vect);
      //g3d_draw_srchball_pos_area(&srchball);
      g3d_draw_srchball_pos_area(srchball.position[0], srchball.position[1],srchball.distMin,srchball.distMax);
    }

  for (i=0; i<lstvert.nv; i++)
    {
      g3d_drawSphere(lstvert.vertex[i].pos[0]+ox,lstvert.vertex[i].pos[1]+oy,lstvert.vertex[i].pos[2]+oz,0.01, tRed, NULL);

    }
 // if (PSP_DRAW_QS)
  //  psp_draw_confs();
}

void psp_draw_search_ball(psp_searchball *srchballpt)
{

  if (sphereActive)
    {
      glPushMatrix();
      g3d_drawSphere(srchballpt->position[0],srchballpt->position[1],srchballpt->position[2],0.1,Green, NULL);
      //g3d_draw_srchball_pos_area(srchballpt);
      glPopMatrix();
    }
  
}

void  psp_draw_in_perspwin()
{

  psp_draw_search_ball(&srchball);
  

}



static double psr_preference_obj(p3d_vector4 obj, double disttocenter, p3d_vector4 porigin)
{
  double w1 = 0.6;
  double w2 = 0.4;
  
  //return (w1/Disttoconeline(obj)) + (w2/Disttoconeorigin(obj))
  return (w1/disttocenter) + (w2/DISTANCE3D(obj[0],obj[1],obj[2],porigin[0],porigin[1],porigin[2]));
    //return 1.0;

}



int psp_init_bitmap_grids()
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int dimx,dimy,dimz;
  int state;


      if(BTSET != NULL)
	hri_bt_destroy_bitmapset(BTSET);
		
      dimx  = (int)((env->box.x2 - env->box.x1)/BT_SAMPLING);
      dimy  = (int)((env->box.y2 - env->box.y1)/BT_SAMPLING);
      dimz  = 1;
      BTSET = hri_bt_create_bitmaps();
      state = hri_bt_init_bitmaps(BTSET,dimx,dimy,dimz,BT_SAMPLING);
      if (state)
	{
	  hri_bt_change_bitmap_position(BTSET,env->box.x1,env->box.y1,
					BTSET->robot->joints[ROBOTj_BASE]->dof_data[2].v);
	  return TRUE;
	}

  return FALSE;
}


