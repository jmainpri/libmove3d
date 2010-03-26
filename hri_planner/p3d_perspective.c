#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Rrt-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "GL/glx.h"
#include "forms.h"
#include "glcanvas.h"
#ifdef __cplusplus
}
#endif

static psp_lst_vertex lstvert;
//static int  vertIndex;
static fct_psp_task_eval task_eval=NULL;
static double pspTaksMaxCost,pspTaksMaxQual;
static void* taskTarget=NULL;
static double lx1, ly1, lz1, lx2, ly2, lz2,lrad;
//static double ox=-1.8,oy=-0.57,oz=0.59;
static double ox=0.0,oy=-0.0,oz=0.0;
static int sphereActive=0;
psp_searchball srchball;
static p3d_jnt *globaljnt = NULL;
static hri_gik * PSP_GIK  = NULL;
static hri_gik * PSP_GIK2 = NULL;
static hri_gik * PSP_GIK3 = NULL;

static configPt * theqs = NULL;
//static int  qindAnt  = 0;
static int  qindex   = 0;
static int  testedQs = 0;
static int  PSP_DRAW_QS = FALSE;
//static double *testedX,*testedY;
static int destIndex = -1;
static double MaxUtil_Res = 1.58;
//static int PSP_GIK_CONTINUOUS = TRUE;
extern int PSP_SRCH_MTD[3];
extern double PSP_PS_TRSHLD;
extern double PSP_DIST2OBJ_TRSHLD;
extern double PSP_DIFF_PS_TRSHLD;
extern int PSP_NEXT_TASK;
extern int PSP_STOP_DEEP;
extern int G3D_RESFRESH_PERSPECTIVE;
extern int PSP_REFRESH_BLOCK;
extern int PSP_NHP_TRAJ;
extern int PSP_RES_counter_max;
extern int PSP_RES_avg_tested;
extern int PSP_RES_avg_found;
extern float PSP_RES_avg_time;
extern double PSP_RES_avg_utility;

p3d_rob *PSP_ROBOT;

int PSP_init_grid;

int PSP_DEACTIVATE_AUTOHIDE;

int PSP_NUM_OBJECTS;
int PSP_CURR_DRAW_OBJ;

int PSP_DRAW_OBJ_ARRAY [PSP_MAX_OBJ_NUM];
float PSP_DRAW_OBJ_COL_INDEX [PSP_MAX_OBJ_NUM];

FILE *pspResultsFile = NULL;
//extern point_co_ordi intersection_point;
//int PSP_DEACTIVATE_AUTOHIDE = 1;
/*******************************************************************************************************************************************************/

/* --------  Perspective Placement functions --------*/

/* List of points generator functions*/
static void     psp_gen_rand_point                    (p3d_vector4 rpoint,double  xMaxLim, double xMinLim, double  yMaxLim, double yMinLim);
static void     psp_gen_rand_3Dpoint                  (p3d_vector4 rpoint, p3d_vector4 center, double Rhomin, double Rhomax);
//static void     psp_gen_ordered_point_list_obj        (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
//static void     psp_gen_point_list_obj        (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex_tmp *lstVtx, hri_bitmapset* PSP_BTSET);
//static void     psp_gen_ordered_point_list_searchball (psp_searchball *sball, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_gen_ordered_spheric_point_list    (p3d_obj *object, p3d_rob *r, int numpoints, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_gen_points                        (int search_method, p3d_rob *obr, p3d_rob *r, int numpoints, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
static void     psp_order_point_list                  (psp_lst_vertex *lstVtx);
static int      psp_get_next_ordered_point            (p3d_vector4 rpoint, psp_lst_vertex *lstVtx);
static int      psp_get_next_ordered_point_tmp            (p3d_vector4 rpoint, psp_lst_vertex_tmp *lstVtx);
static int      psp_get_next_random_point             (p3d_vector4 rpoint, int numlays, int numsegs, psp_lst_vertex *lstVtx );
static double psp_get_point_utility(double quality, double cost);

/* Inverse Kinematics and Configuration test functions */
static int      psp_look_at              (p3d_rob* r, double x, double y, double z, configPt* resq);
static int      psp_look_in_two_times_at (p3d_rob* r, double fromx, double fromy, double fromz, double tox, double toy, double toz, configPt* resq);
static double   psp_test_qs              (p3d_rob *r, configPt q1, configPt q2, p3d_vector4 point , double viewPercent, int checkTraj, hri_bitmapset* PSP_BTSET);

/* Searching Placement Point Functions*/
int  psp_srch_rnd_model_pt       (p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET);
int  psp_srch_model_pt           (p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET);
int  psp_srch_3D_model_pt_obj    (p3d_rob* r, p3d_obj* object, int numpoints, int OnSurface, hri_bitmapset* PSP_BTSET);
//int  psp_srch_model_pt_obj       (p3d_rob* r, p3d_obj* object, int numpoints, double viewPercent, hri_bitmapset* PSP_BTSET);
//int  psp_srch_model_pt_obj       (p3d_rob* r, p3d_obj* object, int numsegs, int numlayers, double viewPercent, hri_bitmapset* PSP_BTSET);
//int  psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r,  int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET);
int  psp_srch_for_target_obj     (p3d_rob *robot, int numsegs, int numlayers, int searchMode, int *searchMtd, double viewpercent, hri_bitmapset* PSP_BTSET);

/* --------- Perspective Observation functions ---------*/

static double pso_watch2_obj();
static double pso_watch_obj();
static int    pso_see_obj();
static int    pso_look_obj();
static double pso_perceive_obj();
//static double pso_watch3_obj();
static int pso_watch_multi_obj(int numObj,double *percentages, p3d_obj **oList);
/* --------- Perspective Reasoning functions  ---------*/ // Testing at 23/08/07

static int    psr_human_look_at (p3d_rob* human, double x, double y, double z);
static double psr_preference_obj(p3d_vector4 obj, double disttocenter, p3d_vector4 porigin);

void psr_get_obj_list(p3d_rob *currRob, p3d_obj **oList, int *nObj,  p3d_rob **rList, int *nRob, double viewPercent);
void psr_get_human_left_pointing             (p3d_rob* human, p3d_rob* r, hri_bitmapset* PSP_BTSET);
void psr_get_human_pointing_from_joint_number(p3d_rob* human, p3d_rob* r, int jntIdx, hri_bitmapset* PSP_BTSET);
void psr_get_pointing_from_joint             (p3d_rob* r,  p3d_jnt *jntPt, int frameType, hri_bitmapset* PSP_BTSET);


/* --------- Utility  Section -------------- */

/* Testing and interface */
void  psp_draw_search_ball      (psp_searchball *srchballpt);
void  psp_draw_in_perspwin      ();
void  psp_search_for_objectives (p3d_rob *robot, p3d_vector3 point);
void  psp_add_element           (psp_lst_elements *lstel, psp_obs_element *elem);
void  g3d_psp_draw_lookatpoint  (p3d_rob *robot);
static void psu_set_num_obj_255();
static  int psp_set_vert2rconf(p3d_matrix4 matr,  psp_obs_vertex *vert, configPt qres);
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

static void     psp_init_lst_vertex(psp_lst_vertex *lstVtx, int numSegs, int numLays);


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
static void psp_init_lst_vertex_obj(psp_lst_vertex *lstVtx,  int numSegs, int numLays);
static void psp_free_lst_vertex(psp_lst_vertex *lstVtx);

static void psu_reboot_theqs(p3d_rob *r, int numqs);

static void  psp_order_point_list_tmp (psp_lst_vertex_tmp *lstVtx);
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

static void psp_get_RTH_to_XY_rframe(p3d_rob* r, p3d_vector4 rpoint, p3d_vector4 outpoint)
{
  double x,y;
  p3d_vector4 point, v_aux;
  //rpoint 0 radius , 1 theta
  x = rpoint[0] * cos(rpoint[1]);
  y = rpoint[0] * sin(rpoint[1]);
  point[0] = x;
  point[1] = y;
  point[2] = 0.0;
  point[3] = 1.0;

  p3d_matvec4Mult(r->joints[1]->abs_pos, point, v_aux);

  rpoint[0] = point[0];
  rpoint[1] = point[1];
  rpoint[2] = point[2];
  rpoint[3] = point[3];

  outpoint[0] = v_aux[0];
  outpoint[1] = v_aux[1];
  outpoint[2] = v_aux[2];
  outpoint[3] = v_aux[3];
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
  double costaux, obsPeraux,utilaux;
  int statusaux, swapped;
  int segaux, layaux;
  p3d_vector4 rvertex;

  for (i=lstVtx->nv-1;i>=0; i--)
  {
    swapped = 0;
    for (j=0; j<i;j++)
    {
      if (lstVtx->vertex[j].cost > lstVtx->vertex[j+1].cost)
	    {
	      //idaux      = lstVtx->vertex[j].id;
	      segaux     = lstVtx->vertex[j].segment;
	      layaux     = lstVtx->vertex[j].layer;
	      rvertex[0] = lstVtx->vertex[j].pos[0];
	      rvertex[1] = lstVtx->vertex[j].pos[1];
	      rvertex[2] = lstVtx->vertex[j].pos[2];
	      costaux    = lstVtx->vertex[j].cost;
	      statusaux  = lstVtx->vertex[j].status;
	      obsPeraux  = lstVtx->vertex[j].quality;
	      utilaux    = lstVtx->vertex[j].utility;
	      lstVtx->vertex[j].id         = j;
	      lstVtx->vertex[j].segment    = lstVtx->vertex[j+1].segment;
	      lstVtx->vertex[j].layer      = lstVtx->vertex[j+1].layer;
	      lstVtx->vertex[j].pos[0]     = lstVtx->vertex[j+1].pos[0];
	      lstVtx->vertex[j].pos[1]     = lstVtx->vertex[j+1].pos[1];
	      lstVtx->vertex[j].pos[2]     = lstVtx->vertex[j+1].pos[2];
	      lstVtx->vertex[j].cost       = lstVtx->vertex[j+1].cost;
	      lstVtx->vertex[j].status     = lstVtx->vertex[j+1].status;
	      lstVtx->vertex[j].quality    = lstVtx->vertex[j+1].quality;
	      lstVtx->vertex[j].utility    = lstVtx->vertex[j+1].utility;
	      lstVtx->grid[lstVtx->vertex[j].segment][lstVtx->vertex[j].layer].id = j;

	      lstVtx->vertex[j+1].id         = j+1;
	      lstVtx->vertex[j+1].segment    = segaux;
	      lstVtx->vertex[j+1].layer      = layaux;
	      lstVtx->vertex[j+1].pos[0]     = rvertex[0];
	      lstVtx->vertex[j+1].pos[1]     = rvertex[1];
	      lstVtx->vertex[j+1].pos[2]     = rvertex[2];
	      lstVtx->vertex[j+1].cost       = costaux;
	      lstVtx->vertex[j+1].status     = statusaux;
	      lstVtx->vertex[j+1].quality    = obsPeraux;
	      lstVtx->vertex[j+1].utility    = utilaux;
	      lstVtx->grid[lstVtx->vertex[j+1].segment][lstVtx->vertex[j+1].layer].id = j+1;

	      swapped = 1;

	    }
    }
    if (!swapped) return;
  }

}




static void  psp_order_point_list_tmp (psp_lst_vertex_tmp *lstVtx)
{
  int i,j;
  double costaux;//, obsPeraux;
  int statusaux, swapped;
  //int segaux, layaux;
  p3d_vector4 rvertex;

  for (i=lstVtx->nv-1;i>=0; i--)
  {
    swapped = 0;
    for (j=0; j<i;j++)
    {
      if (lstVtx->vertex[j].cost > lstVtx->vertex[j+1].cost)
	    {
	      //idaux      = lstVtx->vertex[j].id;
	      rvertex[0] = lstVtx->vertex[j].pos[0];
	      rvertex[1] = lstVtx->vertex[j].pos[1];
	      rvertex[2] = lstVtx->vertex[j].pos[2];
	      costaux    = lstVtx->vertex[j].cost;
	      statusaux  = lstVtx->vertex[j].status;

	      //lstVtx->vertex[j].id     = lstVtx->vertex[j+1].id;
	      lstVtx->vertex[j].pos[0]     = lstVtx->vertex[j+1].pos[0];
	      lstVtx->vertex[j].pos[1]     = lstVtx->vertex[j+1].pos[1];
	      lstVtx->vertex[j].pos[2]     = lstVtx->vertex[j+1].pos[2];
	      lstVtx->vertex[j].cost       = lstVtx->vertex[j+1].cost;
	      lstVtx->vertex[j].status     = lstVtx->vertex[j+1].status;

	      //lstVtx->vertex[j+1].id     = idaux;
	      lstVtx->vertex[j+1].pos[0]     = rvertex[0];
	      lstVtx->vertex[j+1].pos[1]     = rvertex[1];
	      lstVtx->vertex[j+1].pos[2]     = rvertex[2];
	      lstVtx->vertex[j+1].cost       = costaux;
	      lstVtx->vertex[j+1].status     = statusaux;


	      swapped = 1;

	    }
    }
    if (!swapped) return;
  }

}




/****************************************************************/
/*!
 * \brief
 Gets the cost of a point inside the wave propagation matrix and add it to
 the additional costs
 *
 * \param coords -> coordinates of the point to look for
 * \param obr  -> Robot
 * \param angIndex -> current angle index
 * \param maxAngle -> angle limit
 * \param PSP_BTSET -> HRI Bitmap set
 * !

 */
/****************************************************************/

static float psp_get_point_cost(p3d_vector4 coords, p3d_rob *obr, float angIndex,  hri_bitmapset* PSP_BTSET)
{
  double percCost1, percCost2, btcost, totalcost;
  double lamdaD = PSP_DIFF_PS_TRSHLD;
  double lamdaF = 100.0-lamdaD;
  p3d_vector4  rvertex;
  //double gainDist= 0.6;
  p3d_jnt *jntPt = obr->joints[1];
  int x,y,z=0;
  // p3d_vector4 area1,area2;
  //p3d_vector4 point1,point2;
  long maxWaveCost;
  p3d_matvec4Mult(jntPt->abs_pos,coords,rvertex); //obtaining global coordinates

  percCost1 = 0.0;
  percCost2 = 0.0;

  percCost1 = iget_all_wave_cost(rvertex[0], rvertex[1]);

  if (percCost1<0)
  {
    //printf("%f,%f Not in the grid %f\n",rvertex[0], rvertex[1], percCost1);
    return percCost1;
  }

  maxWaveCost =  getMaxWaveCost();
  //changing max meassure for only the max on the area of the robot
  /*    point1[0] = obr->max_pos_range;
   point1[1] = obr->max_pos_range;
   point1[2] = 0.0;
   point1[3] = 1.0;
   point2[0] = 0.0;
   point2[1] = -obr->max_pos_range;
   point2[2] = 0.0;
   point2[3] = 1.0;
   p3d_matvec4Mult(obr->joints[1]->abs_pos,point1,area1);
   p3d_matvec4Mult(obr->joints[1]->abs_pos,point2,area2);
   maxWaveCost = wv_getMaxWaveCostOf(point1[0],point1[1],point2[0],point2[1]);*/

  //////percCost2 =  (angIndex * (1-gainDist)) / maxAngle;
  // percCost1 = (fabs(angleLimit(angIndex)));
  percCost2 = (fabs(angleLimit (angIndex))*lamdaF)/M_PI;



  if (percCost2>lamdaF)
    percCost2 =lamdaF;

  percCost1 = (percCost1 * lamdaD)/ maxWaveCost;
  percCost1 = percCost1 + percCost2;

  if (!p3d_is_in_pos_area (obr,coords[0],coords[1], FALSE))
  {
    //lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
    percCost1 *=1.3;
  }

  if (percCost1>100)
    percCost1 = 100.0;

  //cost of bitmap
  x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
  y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

  if ( rvertex[0] >=  obr->env->box.x1 && rvertex[0] <= obr->env->box.x2 &&  rvertex[1] >=  obr->env->box.y1 && rvertex[1] <= obr->env->box.y2 &&  hri_bt_get_cell(PSP_BTSET->bitmap[BT_OBSTACLES],x,y,z)!=NULL)
  {
    btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
    if (btcost == -2)
    {
      return -3;
    }
    else
    {
      btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
      if (btcost == -2)
        return -3;
      if (btcost>100)
        btcost=100;
    }
  }
  else
  {
    return -3;
  }

  percCost1  = (percCost1 * 50)/100.0;
  percCost2  = (btcost * 50)/100.0;
  //printf("...............Costs bitmap(%i,%i) %f -->  %f / %f  \n",x,y,btcost,percCost2, percCost1);

  totalcost  =  percCost1 + percCost2;
  //printf("%f %f %f\n",rvertex[0], rvertex[1], totalcost);
  if (totalcost>100)
    totalcost = 100;
  return totalcost;
}



/****************************************************************/
/*!
 * \brief Gets the cost of a point inside the wave propagation matrix.
 *
 * \param coords -> coordinates of the point to look for
 * \param obr  -> Robot
 * \param angIndex -> current angle index
 * \param angIndex -> angle limit
 * \param PSP_BTSET -> HRI Bitmap set
 * !

 */
/****************************************************************/
static float psp_get_Point_Cost(p3d_vector4 coords, p3d_rob *obr, float angIndex, float maxAngle, hri_bitmapset* PSP_BTSET)
{
  double percCost1, percCost2, btcost, totalcost;

  p3d_vector4  rvertex;
  //double gainDist= 0.6;
  p3d_jnt *jntPt = obr->joints[1];
  int x,y,z=0;
  long maxWaveCost;
  p3d_matvec4Mult(jntPt->abs_pos,coords,rvertex); //obtaining global coordinates

  percCost1 = 0.0;
  percCost2 = 0.0;

  percCost1 = iget_wave_cost(rvertex[0], rvertex[1]);

  if (percCost1<0)
  {
    //printf("%f,%f Not in the grid %f\n",rvertex[0], rvertex[1], percCost1);
    return percCost1;
  }
  maxWaveCost =  getMaxGridCost();

  //////percCost2 =  (angIndex * (1-gainDist)) / maxAngle;
  // percCost1 = (fabs(angleLimit(angIndex)));
  percCost2 = (fabs(angleLimit (angIndex))*40)/M_PI;
  percCost1 = (percCost1 * 60)/ maxWaveCost;
  percCost1 = percCost1 + percCost2;

  if (!p3d_is_in_pos_area (obr,coords[0],coords[1], FALSE))
  {
    //lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
    percCost1 +=6.0;
  }

  //cost of bitmap
  x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
  y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

  if ( rvertex[0] >=  obr->env->box.x1 && rvertex[0] <= obr->env->box.x2 &&  rvertex[1] >=  obr->env->box.y1 && rvertex[1] <= obr->env->box.y2 &&  hri_bt_get_cell(PSP_BTSET->bitmap[BT_OBSTACLES],x,y,z)!=NULL)
  {
    //printf("................BEFORE OBSTACLES %i %i  \n",x,y);
    btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
    //printf("...............PASSED OBSTACLES %f %f  \n",x,y);
    if (btcost == -2)
    {
      //printf("Not in bitmap \n");
      return -3;
      //lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
    }
    else
    {
      /*if (btcost == -1)
       {
       btcost = 100;
       //lstVtx->vertex[contVert].status = PSP_St_OBS_HIGH_COST;
       }
       else
       {*/
      btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
      //lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
      if (btcost == -2)
        return -3;

    }
  }
  else
  {
    //printf("Not in environment \n");
    return -3;
  }

  percCost1  = (percCost1 * 30)/100.0;
  percCost2  = (btcost * 70)/100.0;
  //printf("...............Costs bitmap(%i,%i) %f -->  %f / %f  \n",x,y,btcost,percCost2, percCost1);

  totalcost  =  percCost1 + percCost2;
  //printf("%f %f %f\n",rvertex[0], rvertex[1], totalcost);
  if (totalcost>100)
    totalcost = 100;
  return totalcost;
}


/****************************************************************/
/*!
 * \brief Gets the cost of a point inside the wave propagation matrix.
 *
 * \param coords -> coordinates of the point to look for
 * \param obr  -> Robot
 * \param angIndex -> current angle index
 * \param angIndex -> angle limit
 * \param PSP_BTSET -> HRI Bitmap set
 * !

 */
/****************************************************************/
static float psp_get_Point_Cost_obj(p3d_vector4 coords, p3d_obj *ob, hri_bitmapset* PSP_BTSET)
{
  double percCost1, percCost2, btcost, totalcost;

  p3d_vector4  rvertex, objCenter;
  int x,y,z=0;
  long maxWaveCost;

  //p3d_matvec4Mult(ob->opos,coords,rvertex); //obtaining global coordinates
  p3d_get_object_center(ob, objCenter);
  rvertex[0] = coords[0]+objCenter[0];
  rvertex[1] = coords[1]+objCenter[1];
  rvertex[2] = coords[2]+objCenter[2];
  rvertex[3] = 1.0;


  //Distance Cost
  percCost1 = iget_all_wave_cost(rvertex[0], rvertex[1]);
  if (percCost1<0)
  {
    return percCost1;
  }
  maxWaveCost =  getMaxWaveCost();

  //Bitmap Cost
  x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
  y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

  if ( rvertex[0] >=  ob->env->box.x1 && rvertex[0] <= ob->env->box.x2 &&  rvertex[1] >=  ob->env->box.y1 && rvertex[1] <= ob->env->box.y2 &&  hri_bt_get_cell(PSP_BTSET->bitmap[BT_OBSTACLES],x,y,z)!=NULL)
  {
    btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
    if (btcost == -2)
    {
      return -3;
    }
    else
    {
      btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
      if (btcost == -2)
        return -3;
      if (btcost > 100)
        btcost = 100;

    }
  }
  else
  {
    return -3;
  }

  /////Total Cost
  percCost1  = (percCost1 * 50)/maxWaveCost;
  percCost2  = (btcost * 50)/100.0;
  //printf("...............Costs bitmap(%i,%i) %f -->  %f / %f  \n",x,y,btcost,percCost2, percCost1);
  totalcost  =  percCost1 + percCost2;
  return totalcost;
}

/****************************************************************/
/*!
 * \brief Gets the cost of a point inside the wave propagation matrix.
 *
 * \param coords -> coordinates of the point to look for
 * \param obr  -> Robot
 * \param angIndex -> current angle index
 * \param angIndex -> angle limit
 * \param PSP_BTSET -> HRI Bitmap set
 * !

 */
/****************************************************************/
static float psp_get_Point_Cost_searchball(p3d_vector4 coords, psp_searchball *sball, hri_bitmapset* PSP_BTSET)
{
  double percCost1, percCost2, btcost, totalcost;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4  rvertex;
  int x,y,z=0;
  long maxWaveCost;

  rvertex[0] = coords[0]+sball->position[0];
  rvertex[1] = coords[1]+sball->position[1];
  rvertex[2] = coords[2]+sball->position[2];
  rvertex[3] = 1.0;

  //Distance cost
  percCost1 = iget_all_wave_cost(rvertex[0], rvertex[1]);

  if (percCost1<0)
  {
    return percCost1;
  }
  maxWaveCost =  getMaxWaveCost();

  //Bitmap Cost
  x = (rvertex[0] - PSP_BTSET->realx)/ PSP_BTSET->pace;
  y = (rvertex[1] - PSP_BTSET->realy)/ PSP_BTSET->pace;

  if ( rvertex[0] >=  env->box.x1 && rvertex[0] <= env->box.x2 &&  rvertex[1] >=  env->box.y1 && rvertex[1] <= env->box.y2 &&  hri_bt_get_cell(PSP_BTSET->bitmap[BT_OBSTACLES],x,y,z)!=NULL)
  {
    btcost = PSP_BTSET->bitmap[BT_OBSTACLES]->data[x][y][z].val;
    if (btcost == -2)
    {
      return -3;
    }
    else
    {
      btcost = PSP_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(PSP_BTSET,x,y,z);
      if (btcost == -2)
        return -3;
      if (btcost > 100)
        btcost = 100;
    }
  }
  else
  {
    //Not in environment
    return -3;
  }


  /////// Total cost 50-50

  percCost1  = (percCost1 * 50)/maxWaveCost;
  percCost2  = (btcost * 50)/100.0;
  //printf("...............Costs bitmap(%i,%i) %f -->  %f / %f  \n",x,y,btcost,percCost2, percCost1);
  totalcost  =  percCost1 + percCost2;
  return totalcost;
}


/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost in a complete way
 *
 * \param or -> objetctif
 * \param r  -> Robot
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/


static void  psp_gen_ordered_point_list_complete (p3d_rob *obr, p3d_rob *r, int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  //double distance = ((obr->max_pos_range -  obr->min_pos_range) / 2.0) + obr->min_pos_range ;
  double distance = obr->min_pos_range;
  double layerStep;
  p3d_vector4 auxpoint;//, rvertex;
  p3d_jnt *jntPt = obr->joints[1];
  //p3d_matrix4 matrix;
  int segIdx;
  double angle;
  int i,j;//,x,y,z=0;
  int contVert = 0;
  int contSegs = 0;
  double maxAngle;//, maxDist = linearDistance(jntPt->p0.x ,jntPt->p0.y,r->joints[1]->p0.x ,r->joints[1]->p0.y) + distance;
  double percCost1;//, percCost2, btcost;

  //double gainDist= 0.6;
  globaljnt = jntPt;

  if (numlayers>1)
    layerStep = ((obr->max_pos_range -  obr->min_pos_range) / (numlayers-1.0));
  else
    layerStep = 0;

  lstVtx->currentVert = 0;

  /* for(i=0 ; i<=3 ; i++){
   for(j=0 ; j<=3 ; j++){
   matrix[i][j]=jntPt->abs_pos[i][j];
   }
   }*/

  auxpoint[3]  = 1.0;
  //matrix[2][3] = 0.0;

  maxAngle   = numsegs/2;
  //printf("--------- VERTEX COSTS -------------");
  for (i=0; i<=numsegs/2; i++)
  {

    angle  = (2*M_PI)*i/(numsegs*2);
    auxpoint[2] = 0.0;
    segIdx =(int) maxAngle - i;
    printf("segment %i  = %i\n",i,segIdx);
    for (j=0; j<numlayers; j++)
    {
      distance =  obr->min_pos_range+(j*layerStep);
      //	  printf("distance %i,%i = %f\n",i,j,distance);
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);

      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];
      lstVtx->vertex[contVert].id = contVert;
      lstVtx->vertex[contVert].segment = segIdx;
      lstVtx->vertex[contVert].layer   = j;
      lstVtx->vertex[contVert].quality = 0;
      lstVtx->vertex[contVert].utility = 0;

      lstVtx->grid[segIdx][j].id = contVert;
      lstVtx->grid[segIdx][j].pos[0] = auxpoint[0];
      lstVtx->grid[segIdx][j].pos[1] = auxpoint[1];
      lstVtx->grid[segIdx][j].pos[2] = auxpoint[2];
      lstVtx->grid[segIdx][j].quality = 0;
      lstVtx->grid[segIdx][j].utility = 0;
      ////////////// Costs

      //percCost1 = psp_get_Point_Cost(auxpoint, obr, angle , maxAngle, PSP_BTSET);
      percCost1 = psp_get_point_cost(auxpoint, obr, angle , PSP_BTSET);
      if (percCost1>=0)
	    {
	      lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      lstVtx->grid[segIdx][j].status = PSP_NON_OBSERVED;
	      //printf("observable %i\n",i);
	    }
      else
	    {
	      percCost1 = -1;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      lstVtx->grid[segIdx][j].status = PSP_NOT_AVAILABLE;
	      //printf("NON observable %i\n",i);
	    }

      lstVtx->grid[segIdx][j].cost = lstVtx->vertex[contVert].cost   =  percCost1;

      contVert++;
    }

    contSegs++;
  }
  //segIdx=numSegs;
  double angStep = M_PI/numsegs;
  maxAngle = numsegs*2;// - (numSegs*1.5);
                       //for (i+=numsegs-1; i<numsegs*2; i++)
  for (i=0; i<numsegs/2; i++)
  {
    //angle  = (2*M_PI)*i/(numsegs*2);
    angle  = ((1.5*M_PI)+(i*angStep));

    auxpoint[2] = 0.0;
    segIdx = (numsegs/2) + ((int) maxAngle - (i+(numsegs*1.5)));
    printf("segment %i  = %i\n",i,segIdx);
    for (j=0; j<numlayers; j++)
    {
      distance =  obr->min_pos_range+(j*layerStep);
      //printf("distance %i,%i = %f\n",i,j,distance);
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      //auxpoint[0] = (distance+(j*layerStep)) * cos(angle);
      //auxpoint[1] = (distance+(j*layerStep)) * sin(angle);
      //printf("capa %i %f,%f\n",j,auxpoint[0],auxpoint[1]);
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];
      lstVtx->vertex[contVert].id = contVert;
      lstVtx->vertex[contVert].segment = segIdx;
      lstVtx->vertex[contVert].layer   = j;
      lstVtx->vertex[contVert].quality = 0;
      lstVtx->vertex[contVert].utility = 0;

      lstVtx->grid[segIdx][j].id = contVert;
      lstVtx->grid[segIdx][j].pos[0] = auxpoint[0];
      lstVtx->grid[segIdx][j].pos[1] = auxpoint[1];
      lstVtx->grid[segIdx][j].pos[2] = auxpoint[2];
      lstVtx->grid[segIdx][j].quality = 0;
      lstVtx->grid[segIdx][j].utility = 0;
      percCost1=0;
      ////////////// Costs

      //percCost1 = psp_get_Point_Cost(auxpoint, obr, angle , maxAngle, PSP_BTSET);
      percCost1 = psp_get_point_cost(auxpoint, obr, angle , PSP_BTSET);
      if (percCost1>=0)
	    {
	      lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      lstVtx->grid[segIdx][j].status = PSP_NON_OBSERVED;
	      //printf("observable %i\n",i);
	    }
      else
	    {
	      percCost1 = -1;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      lstVtx->grid[segIdx][j].status = PSP_NOT_AVAILABLE;
	      //printf("NON observable %i\n",i);
	    }
      //printf("cost %f: \n",percCost1);
      lstVtx->grid[segIdx][j].cost = lstVtx->vertex[contVert].cost   =  percCost1;

      contVert++;
    }
    contSegs++;
  }

  printf("----- total vertex  %i ------- \n",contVert);
  lstVtx->nv = contVert;
  lstVtx->ns = contSegs;//numsegs+2;
  lstVtx->nl = numlayers;
  //psp_order_point_list(lstVtx);
  printf("segs before %i  after %i\n", numsegs, contSegs);
}


/****************************************************************/
/*!
 * \brief Generates the list of ordered points with determined cost for a robot, in a complete circle
 *
 * \param object -> object
 * \param r  -> Robot
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_ordered_point_list_around (p3d_rob *obr, p3d_rob *r, int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance = obr->min_pos_range;
  double layerStep;
  p3d_vector4 auxpoint;//, rvertex;
  p3d_jnt *jntPt = obr->joints[1];
  //p3d_matrix4 matrix;
  int segIdx;
  double angle;
  int i,j;//,x,y,z=0;
  int contVert = 0;
  double maxAngle;//, maxDist = linearDistance(jntPt->p0.x ,jntPt->p0.y,r->joints[1]->p0.x ,r->joints[1]->p0.y) + distance;
  double percCost1;//, percCost2, btcost;

  //double gainDist= 0.6;
  globaljnt = jntPt;

  if (numlayers>1)
    layerStep = ((obr->max_pos_range -  obr->min_pos_range) / (numlayers-1.0));
  else
    layerStep = 0;

  lstVtx->currentVert = 0;

  /* for(i=0 ; i<=3 ; i++){
   for(j=0 ; j<=3 ; j++){
   matrix[i][j]=jntPt->abs_pos[i][j];
   }
   }*/

  auxpoint[3]  = 1.0;
  //matrix[2][3] = 0.0;

  maxAngle   = numsegs/2;
  //printf("--------- VERTEX COSTS -------------");
  for (i=0; i<=numsegs; i++)
  {

    angle  = (2*M_PI)*i/(numsegs*2);
    auxpoint[2] = 0.0;
    segIdx =(int) maxAngle - i;
    for (j=0; j<numlayers; j++)
    {
      distance =  obr->min_pos_range+(j*layerStep);
      //	  printf("distance %i,%i = %f\n",i,j,distance);
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);
      //printf("capa %i %f,%f\n",j,auxpoint[0],auxpoint[1]);
      lstVtx->vertex[contVert].pos[0] = auxpoint[0];
      lstVtx->vertex[contVert].pos[1] = auxpoint[1];
      lstVtx->vertex[contVert].pos[2] = auxpoint[2];

      lstVtx->vertex[contVert].segment = segIdx;
      lstVtx->vertex[contVert].layer   = j;
      lstVtx->vertex[contVert].quality = 0;
      lstVtx->vertex[contVert].utility = 0;
      lstVtx->grid[segIdx][j].pos[0] = auxpoint[0];
      lstVtx->grid[segIdx][j].pos[1] = auxpoint[1];
      lstVtx->grid[segIdx][j].pos[2] = auxpoint[2];


      ////////////// Costs

      percCost1 = psp_get_Point_Cost(auxpoint, obr, angle , maxAngle, PSP_BTSET);
      if (percCost1>=0)
	    {
	      lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      lstVtx->grid[segIdx][j].status = PSP_NON_OBSERVED;
	      //printf("observable %i\n",i);
	    }
      else
	    {
	      percCost1 = -1;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      lstVtx->grid[segIdx][j].status = PSP_NOT_AVAILABLE;
	      //printf("NON observable %i\n",i);
	    }

      lstVtx->grid[segIdx][j].cost = lstVtx->vertex[contVert].cost   =  percCost1;

      contVert++;
    }


  }

  printf("----- total vertex  %i ------- \n",contVert);
  lstVtx->nv = contVert;
  lstVtx->ns = numsegs;
  lstVtx->nl = numlayers;
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
  int i,j;//,x,y,z=0;
  int contVert = 0;

  double maxAngle, maxDist = linearDistance(jntPt->p0.x ,jntPt->p0.y,r->joints[1]->p0.x ,r->joints[1]->p0.y) + distance;
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
    percCost1 =  (linearDistance(ps[0],ps[1],  rvertex[0], rvertex[1])*gainDist) / maxDist;
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
    percCost1 =  (linearDistance(ps[0],ps[1], rvertex[0], rvertex[1])*gainDist) / maxDist;
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
 * \brief Generates the list of ordered points depending on the method
 *
 * \param object -> object
 * \param r  -> Robot
 * \param numpoints -> number of points to generate
 * !

 */
/****************************************************************/

static void psp_gen_points (int search_method, p3d_rob *obr, p3d_rob *r, int numpoints, int numlayers,  psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{

  /*    case  PSP_AROUND:
   psp_gen_ordered_point_list_around (obr, r, numpoints, lstVtx, PSP_BTSET);
   break;
   case  PSP_FRONT:
   psp_gen_ordered_point_list (obr, r, numpoints, lstVtx, PSP_BTSET);
   break;
   case PSP_FRONT_WTRAJ:
   psp_gen_ordered_point_list_wtraj (obr, r, numpoints, lstVtx, PSP_BTSET);
   break;
   case  PSP_FRONT_COMPLETE:
   psp_gen_ordered_point_list_complete (obr, r, numpoints, numlayers, lstVtx, PSP_BTSET);
   break;
   */
  switch (search_method)
  {
    case  PSP_AROUND:
      //psp_init_lst_vertex(lstVtx, numpoints , 1);
      psp_gen_ordered_point_list_around (obr, r, numpoints, numlayers, lstVtx, PSP_BTSET);
      break;
    case  PSP_FRONT:
      //psp_init_lst_vertex(lstVtx, numpoints, numlayers);
      psp_gen_ordered_point_list_complete (obr, r, numpoints, numlayers, lstVtx, PSP_BTSET);
      break;
  }

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

static void psp_gen_point_list_obj_complete (p3d_obj *object, p3d_rob *r, int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{
  double distance;// = ((object->max_pos_range -  object->min_pos_range) / 2.0) + object->min_pos_range ;
  p3d_vector4 auxpoint, rvertex, objCenter;
  double layerStep;

  //double ps[3], pg[3];

  double angle;
  int i,j;//,x,y,z=0;
  int contVert = 0;


  p3d_get_object_center(object, objCenter);
  printf("+++++...... on center point %f, %f,%f\n",objCenter[0],objCenter[1],objCenter[2]);
  //double maxAngle, maxDist = linearDistance(objCenter[0], objCenter[1], r->joints[1]->p0.x, r->joints[1]->p0.y);
  double percCost1;//, percCost2, btcost;

  // double gainDist= 0.6;

  if (numlayers>1)
    layerStep = ((object->max_pos_range -  object->min_pos_range) / (numlayers-1.0));
  else
    layerStep = 0;

  //ps[0] = r->joints[1]->abs_pos[0][3];
  //ps[1] = r->joints[1]->abs_pos[1][3];
  //ps[2] = r->joints[1]->abs_pos[2][3];//0.0;

  //pg[2] = r->joints[1]->abs_pos[2][3];//0.0;

  // lstVtx->nv = numpoints;
  lstVtx->currentVert = 0;

  //maxAngle = numpoints/2;
  auxpoint[2] = 0.0;
  for (i=0; i<=numsegs; i++)
  {
    angle  = (2*M_PI)*i/numsegs;
    for (j=0; j<numlayers; j++)
    {
      distance =  object->min_pos_range+(j*layerStep);
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);


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
      percCost1 = psp_get_Point_Cost_obj(auxpoint, object, PSP_BTSET);
      if (percCost1>=0)
	    {
	      lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      //printf("observable %i\n",i);
	    }
      else
	    {
	      percCost1 = -1;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      //printf("NON observable %i\n",i);
	    }

      lstVtx->vertex[contVert].cost  =   percCost1;// + percCost2;


      ///////////////////

	    printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
	    contVert++;
    }
  }

  lstVtx->nv = contVert;
  //printf("List Ordered\n");
  // psp_order_point_list(lstVtx);


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

static void psp_gen_point_list_searchball (psp_searchball *sball, p3d_rob *r,  int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
{

  p3d_vector4 auxpoint, rvertex;
  double distance;
  double layerStep;
  double angle;
  double percCost1;
  int i,j;
  int contVert = 0;



  if (numlayers>1)
    layerStep = ((sball->distMax -  sball->distMin) / (numlayers-1.0));
  else
    layerStep = 0;


  lstVtx->currentVert = 0;

  auxpoint[2] = 0.0;
  for (i=0; i<=numsegs; i++)
  {
    angle  = (2*M_PI)*i/numsegs;
    for (j=0; j<numlayers; j++)
    {
      distance =  sball->distMin+(j*layerStep);
      auxpoint[0] = distance * cos(angle);
      auxpoint[1] = distance * sin(angle);


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
      percCost1 = psp_get_Point_Cost_searchball(auxpoint, sball, PSP_BTSET);
      if (percCost1>=0)
	    {
	      lstVtx->vertex[contVert].status = PSP_St_OBSERVABLE;
	      //printf("observable %i\n",i);
	    }
      else
	    {
	      percCost1 = -1;
	      lstVtx->vertex[contVert].status = PSP_St_NOT_IN_RANGE;
	      //printf("NON observable %i\n",i);
	    }

      lstVtx->vertex[contVert].cost  =   percCost1;// + percCost2;


      ///////////////////

	    //printf("%f %f %f\n",auxpoint[0] ,auxpoint[1],  lstVtx->vertex[contVert].cost );
	    contVert++;
    }
  }

  lstVtx->nv = contVert;



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

  double maxAngle;// maxDist = linearDistance(objCenter[0], objCenter[1], r->joints[1]->p0.x, r->joints[1]->p0.y);
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

            if ( rvertex[0] >=  r->env->box.x1 && rvertex[0] <= r->env->box.x2 &&  rvertex[1] >=  r->env->box.y1 && rvertex[1] <= r->env->box.y2 &&  hri_bt_get_cell( PSP_BTSET->bitmap[BT_OBSTACLES],x,y,z))
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
    lstVtx->currentVert++;
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

static int psp_get_next_ordered_point_tmp(p3d_vector4 rpoint, psp_lst_vertex_tmp *lstVtx )
{

  if(lstVtx->vertex[lstVtx->currentVert].status==PSP_St_NOT_IN_RANGE)
  {
    lstVtx->currentVert++;
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
 * \brief Find the next random point in the list of points
 *
 * \param rpoint - resulting point
 * \param lstvtx - list of points
 * !

 */
/****************************************************************/

static int psp_get_next_random_point(p3d_vector4 rpoint, int numlays, int numsegs, psp_lst_vertex *lstVtx )
{
  p3d_vector4 point1;//, point2, point3;
  int idx = lstVtx->currentVert;
  double dMax,dMin,tMax,tMin;
  int l,s;

  if (idx >= lstVtx->nv)
    return 0;

  s = (int) idx/numlays; //segment
  l = idx % numlays;     //layer

  if (l>=numlays-1)
  {
    idx++;
    s = (int) idx/numlays;
    l = idx % numlays;
  }

  if (s>=numsegs-1)
    return 0;

  dMin = linearDistance(0.0, 0.0, lstVtx->grid[s][l].pos[0],lstVtx->grid[s][l].pos[1]);
  dMax = linearDistance(0.0, 0.0, lstVtx->grid[s+1][l+1].pos[0],lstVtx->grid[s+1][l+1].pos[1]);

  tMax = rad_angleOf(0,0, lstVtx->grid[s][l].pos[0],lstVtx->grid[s][l].pos[1]);
  tMin = rad_angleOf(0,0, lstVtx->grid[s+1][l+1].pos[0],lstVtx->grid[s+1][l+1].pos[1]);

  if (tMin >= tMax && tMax == 0)
    tMax = 2*M_PI;


  psp_gen_rand_point(point1,dMax,dMin,tMax,tMin);
  printf(" >>>>> Random : MaxD %f MinD %f -- tMax %f tMin %f\n",dMax,dMin,tMax,tMin);
  printf(" >>>>> Random Point:  %f %f  between %i %i - %i %i\n", point1[0],point1[1],s,l,s+1,l+1);
  rpoint[0] = point1[0] * cos(point1[1]); // radius * cos(theta)
  rpoint[1] = point1[0] * sin(point1[1]);
  rpoint[2] = 0.0;
  rpoint[3] = 1.1;

  lstVtx->currentVert=idx+1;

  return 1;
}

/****************************************************************/
/*!
 * \brief Find the next random point in the list of points
 *
 * \param rpoint - resulting point
 * \param lstvtx - list of points
 * !

 */
/****************************************************************/

static int psp_get_next_random_point_list(p3d_vector4 rpoint, int numlays, int numsegs, psp_lst_vertex *lstVtx )
{
  //p3d_vector4 point1;//, point2, point3;
  int idx;// = lstVtx->currentVert;
  int contrand = 0;
  double dMax;//,dMin;//,tMax,tMin;
              //  int l,s;

  dMax = 0 + ( (numlays*numsegs-1.0) *rand()/(RAND_MAX + 1.0));
  idx = (int) dMax;
  // printf("index %i - %f\n",idx,dMax);
  while(lstVtx->vertex[idx].status==PSP_St_NOT_IN_RANGE || lstVtx->vertex[idx].status==PSP_St_OBSERVED )
  {
    dMax = 0 + ( (numlays*numsegs-1.0) *rand()/(RAND_MAX + 1.0));
    idx =  (int) dMax;
    contrand ++;
    if (contrand > lstVtx->nv)
      return 0;
    //printf("index %i - %f\n",idx,dMax);
  }


  lstVtx->currentVert=idx;

  //lstVtx->vertex[idx].status = PSP_St_OBSERVED;

  rpoint[0] = lstVtx->vertex[lstVtx->currentVert].pos[0];
  rpoint[1] = lstVtx->vertex[lstVtx->currentVert].pos[1];
  rpoint[2] = lstVtx->vertex[lstVtx->currentVert].pos[2];
  rpoint[3] = 1.0;


  return 1;
}

/****************************************************************/
/*!
 * \brief Finds a robot pan tilt angles for camera
 *
 * \param r - robot
 * \param xo, yo, zo - point
 * \param resq - resulting q
 * !
 */
/* ***************************************************************/

int psp_set_pan_tilt(p3d_rob *r,  p3d_vector4 realcoord, configPt resq)
{
  p3d_vector4 newcoord;
  p3d_matrix4 inv;
  double phi, theta;
  p3d_matInvertXform(r->joints[ROBOTj_TILT]->abs_pos, inv);

  p3d_matvec4Mult(inv, realcoord, newcoord);

  p3d_psp_cartesian2spherical(newcoord[0],newcoord[1],newcoord[2],
                              0,0,0,&phi,&theta);
  //if (phi >
  resq[ROBOTq_TILT] = phi;
  resq[ROBOTq_PAN] = theta;
  p3d_set_and_update_this_robot_conf(r,resq);

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
  p3d_vector3 point2look[3];
  double distGoal = 0.3;
  double Jcoord[6];
  int res=0, res2,i;

  p3d_mat4ExtractPosReverseOrder(r->joints[ROBOTj_LOOK]->abs_pos, Jcoord, Jcoord+1, Jcoord+2,
                                 Jcoord+3, Jcoord+4, Jcoord+5);

  if (DISTANCE3D(x,y,z,Jcoord[0],Jcoord[1],Jcoord[2]) <= distGoal)
  {
    //printf("Already...LOOK\n");
    return TRUE;
  }

#ifdef HRI_JIDO
  //int jointInd= ROBOTj_LOOK;
  int jointindexesR[]= {ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK}; //Jido (Platine1,platine2, look)
#elif defined HRI_TUM_BH
  //int jointInd= 32;
  int jointindexesR[]= {2,4,ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK}; //BH (body, neck, head1, head2, head3, look)
#elif defined HRI_HRP2
  //int jointInd= ROBOTj_LOOK;
  int jointindexesR[]= {14, ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK}; //HRP2 (Head1,Head2, look)
#elif defined HRI_BERT1
  int jointindexesR[]= {14, ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK}; //HRP2 (Head1,Head2, look)
#else
#error Robot platform must be defined !!
#endif
  for (i=0;i<3;i++)
  {
    point2look[i][0]=x;
    point2look[i][1]=y;
    point2look[i][2]=z;
  }
  printf("Searching...LOOK\n");
  if (PSP_GIK != NULL)
  {
    if(!PSP_GIK->GIKInitialized){
#ifdef HRI_JIDO
      /***** FOR JIDO *****/
      hri_gik_initialize_gik(PSP_GIK,r,1,3); //
      hri_gik_add_task(PSP_GIK, 3, 3, 1, jointindexesR, ROBOTj_LOOK);  /* Cameras */
#elif defined HRI_TUM_BH
      /***** FOR BH *****/
      hri_gik_initialize_gik(PSP_GIK,r,1,5); /* Attention to joint number */
      hri_gik_add_task(PSP_GIK, 3, 5, 1, jointindexesR, ROBOTj_LOOK);  /* HEAD */
      //printf("BH\n");
#elif defined HRI_HRP2
      /***** FOR HRP2 *****/
      hri_gik_initialize_gik(PSP_GIK,r,1,4); //
      hri_gik_add_task(PSP_GIK, 3, 4, 1, jointindexesR, ROBOTj_LOOK);  /* HEAD */
#elif defined HRI_BERT1
#else
#error Robot platform must be defined !!
#endif

    }

    // printf("INITIALIZED\n");

    res = hri_gik_compute(r, PSP_GIK, 150, distGoal, 1, 0, point2look,NULL, resq, NULL);
#ifdef HRI_HRP2
    res2 = p3d_col_test_robot(r,2);
    //printf("collision 2:   %i \n",res2);
    set_kcd_which_test(P3D_KCD_ROB_ALL);
    res2 = p3d_col_does_robot_collide(r->num, p3d_numcoll);
    //res2 = kcd_check_report(r->num);//also possible
    //printf("collision 2:   %i \n",res2);
#else
    res2 = p3d_col_test_robot(r,0);
#endif

    g3d_refresh_allwin_active();
    if (res && !res2)
    {
      p3d_set_and_update_this_robot_conf(r,*resq);
      //printf("CONF. FOUND\n");
      return(res);
    }
    else
    {
      //resq = NULL;
      return FALSE;	//printf("NO CONF.FOUND\n");

    }
  }
  // printf("End...LOOK\n");

  return FALSE;

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
  int res=0;
#ifdef HRI_JIDO
  p3d_vector3 point2look;
  p3d_vector4 jointcenter;
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
#endif

  return(res);

}

/****************************************************************/
/*!
 * \brief Finds a robot configuration to reach a point
 *
 * \param r - robot
 * \param point2give - where to place the grip
 * \param resq - resulting q
 * \param quality - Resulting meassure of the feasability of the task
 * \param cost - resulting cost of performing the task
 * !
 */
/****************************************************************/

static int psp_human_take(p3d_rob* r, p3d_vector3 *point2receive, configPt *resq,  double *quality, double *cost)
{
  p3d_vector4 point, pointIni;
  int res=0,res2=0;
  double distConf, dist2Obj;
  configPt qIni =  p3d_get_robot_config(r);



  int gripObject = 30;//RARM_LINK6
  int jointindexesR2[]= {17,18,19};
  //int jointindexesR[]=  {0,  ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK, 0,  0,  0,  0,  0,  0}; //HRP2 (Head1,Head2, look)
  int njoints = 3;



  if (r->joints[gripObject]->o)
    p3d_get_object_center(r->joints[gripObject]->o, pointIni);
  else
    p3d_get_robot_center(r, pointIni);

  ///find a conf
  if (PSP_GIK2 != NULL)
  {
    printf("INITIALIZING ... \n");
    if(!PSP_GIK2->GIKInitialized){
      hri_gik_initialize_gik(PSP_GIK2,r,1,njoints);

      //hri_gik_add_task(PSP_GIK2, 3, njoints, 2, jointindexesR, ROBOTj_LOOK);  /* Look */
      hri_gik_add_task(PSP_GIK2, 3, njoints, 1, jointindexesR2, ROBOTj_GRIP);  /* Place grip */


    }

    res = hri_gik_compute(r, PSP_GIK2, 200, 0.1, 1, 0, point2receive, NULL, resq, NULL);
  }

  if (r->joints[gripObject]->o)
    p3d_get_object_center(r->joints[gripObject]->o, point);
  else
    p3d_get_robot_center(r, point);

  distConf =   p3d_dist_config(r, qIni,*resq);//Distance between original and final Configurations
                                              //distConf +=  DISTANCE3D(pointIni[0],pointIni[1],pointIni[2],point2give[0][0],point2give[0][1],point2give[0][2]);//Original Distance between finger(Object)  and object to reach
  dist2Obj =   DISTANCE3D(point[0],point[1],point[2],point2receive[0][0],point2receive[0][1],point2receive[0][2]);//Final Distance between finger(Object)  and object to reach

  //printf("Distances \n     %f + %f == %f\n",distConf,dist2Obj, distConf+dist2Obj);

  res2 = p3d_col_test_robot(r,0);


  *quality = dist2Obj;
  *cost = distConf;
  if (!res2)
  {
    //find quality

    p3d_set_and_update_this_robot_conf(r,*resq);
    printf("GIVE CONF. FOUND\n");
  }
  else
  {
    //resq = NULL;
    //*quality = -1;
    //*quality = -4;
    res = -3;
    printf("NO GIVE CONF.FOUND\n");

  }
  return res;
}


/****************************************************************/
/*!
 * \brief Finds a robot configuration to reach a point
 *
 * \param r - robot
 * \param point2give - where to place the grip
 * \param resq - resulting q
 * \param quality - Resulting meassure of the feasability of the task
 * \param cost - resulting cost of performing the task
 * !
 */
/****************************************************************/

static int psp_place_grip(p3d_rob* r, p3d_vector3 *point2give, configPt *resq,  double *quality, double *cost, int iter)
{
  p3d_vector4 point, pointIni;
  int res=0,res2=0;
  double distConf, dist2Obj;
  configPt qIni =  p3d_get_robot_config(r);

#ifdef  HRI_JIDO
  int gripObject = 16;//FINGER
  int jointindexesR2[]= {5, 6, 7, 8, 9, 10, 13}; //Jido arm
  int jointArmBase = 16;
  int njoints = 7;
#elif defined HRI_HRP2
  int gripObject = 29;//RARM_LINK6
                      //int jointindexesR2[]= {14, 19, 20, 21, 22, 23, 24};//, 25}; //HRP RIGHT ARM//original
  int jointindexesR2[]= {14, 0,          0,           0,          19, 20, 21, 22, 23, 24};
  int jointindexesR[]=  {0,  ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK, 0,  0,  0,  0,  0,  0}; //HRP2 (Head1,Head2, look)
  int njoints = 7+3;//9
                    //int jointArmBase = 19;
#elif defined HRI_BERT1
  int gripObject = 29;//RARM_LINK6
                      //int jointindexesR2[]= {14, 19, 20, 21, 22, 23, 24};//, 25}; //HRP RIGHT ARM//original
  int jointindexesR2[]= {14, 0,          0,           0,          19, 20, 21, 22, 23, 24};
  int jointindexesR[]=  {0,  ROBOTj_PAN, ROBOTj_TILT, ROBOTj_LOOK, 0,  0,  0,  0,  0,  0}; //HRP2 (Head1,Head2, look)
  int njoints = 7+3;//9
                    //int jointArmBase = 19;
#elif defined HRI_TUM_BH
  int gripObject = 9;//TODO: check this number
  int jointindexesR2[]=  {2,3,8,9,10,11,12}; //BH RIGHT ARM
  int njoints = 7;
  int jointArmBase = 9;
#endif
  if (r->joints[gripObject]->o)
    p3d_get_object_center(r->joints[gripObject]->o, pointIni);
  else
    p3d_get_robot_center(r, pointIni);

  ///find a conf
  if (PSP_GIK2 != NULL)
  {
    printf("INITIALIZING ... \n");
    if(!PSP_GIK2->GIKInitialized){
      hri_gik_initialize_gik(PSP_GIK2,r,1,njoints);
#ifdef  HRI_HRP2
      hri_gik_add_task(PSP_GIK2, 3, njoints, 2, jointindexesR, ROBOTj_LOOK);  /* Look */
#endif
      hri_gik_add_task(PSP_GIK2, 3, njoints, 1, jointindexesR2, ROBOTj_GRIP);  /* Place grip */


    }

    res = hri_gik_compute(r, PSP_GIK2, iter, 0.1, 1, 0, point2give, NULL, resq, NULL);
  }

  if (r->joints[gripObject]->o)
    p3d_get_object_center(r->joints[gripObject]->o, point);
  else
    p3d_get_robot_center(r, point);

  distConf =   p3d_dist_config(r, qIni,*resq);//Distance between original and final Configurations
                                              //distConf +=  DISTANCE3D(pointIni[0],pointIni[1],pointIni[2],point2give[0][0],point2give[0][1],point2give[0][2]);//Original Distance between finger(Object)  and object to reach
  dist2Obj =   DISTANCE3D(point[0],point[1],point[2],point2give[0][0],point2give[0][1],point2give[0][2]);//Final Distance between finger(Object)  and object to reach

  //printf("Distances \n     %f + %f == %f\n",distConf,dist2Obj, distConf+dist2Obj);
#ifdef HRI_HRP2
  res2 = p3d_col_test_robot(r,2);
  set_kcd_which_test(P3D_KCD_ROB_ALL);
  res2 = p3d_col_does_robot_collide(r->num, p3d_numcoll);
  //res = kcd_check_report(gik->robot->num);//also possible
#else
  res2 = p3d_col_test_robot(r,0);
#endif

  *quality = dist2Obj;
  *cost = distConf;
  if (!res2)
  {
    //find quality

    p3d_set_and_update_this_robot_conf(r,*resq);
    printf("GIVE CONF. FOUND\n");
  }
  else
  {
    //resq = NULL;
    //*quality = -1;
    //*quality = -4;
    res = -3;
    printf("NO GIVE CONF.FOUND\n");

  }
  return res;
}

/****************************************************************/
/*!
 * \brief Finds a robot configuration to give an object to a human
 *
 * \param r - robot
 * \param obr - humanor a robot
 * \param resq - resulting q
 * \param quality - Resulting meassure of the feasability of the task
 * \param cost - resulting cost of performing the task
 * !
 */
/****************************************************************/

static int psp_give_to(p3d_rob* r, void* obr, configPt *resq,  double *quality, double *cost)
{

  p3d_vector4 center,point;//, pointIni;
  p3d_vector3 point2give[3];
  int res=0,i;
  //double distConf, dist2Obj;
  //configPt qIni =  p3d_get_robot_config(r);


  //double dist2point;
  p3d_rob* temprob = ((p3d_rob*)obr);
  ///find human receiving point
  center[0] = 0.3; //30cm -- in front of the robot
  center[1] = 0.0;
  center[2] = 0.3;//center[2];//
  center[3] = 1.0;
  p3d_matvec4Mult(temprob->joints[1]->abs_pos, center, point);

  for (i=0;i<3;i++)
  {
    ox = point2give[i][0] = point[0];
    oy = point2give[i][1] = point[1];
    oz = point2give[i][2] = point[2];
  }

#ifdef HRI_HRP2
  p3d_get_object_center(temprob->o[temprob->cam_body_index], center);
  point2give[1][0] = center[0];
  point2give[1][1] = center[1];
  point2give[1][2] = center[2];
#endif

  res = psp_place_grip(r, &point2give[0], resq,  quality, cost, 200);

  return(res);

}

/****************************************************************/
/*!
 * \brief Finds a robot configuration to give an object to a human
 *
 * \param r - robot
 * \param obr - human
 * \param resq - resulting q
 * \param quality - Resultin messure of the feasability of the task
 * !
 */
/****************************************************************/

static int psp_take_from_surface(p3d_rob* r, p3d_obj* obj, configPt *resq,  double *quality)
{
  p3d_vector4 center,point, pointIni;
  p3d_vector3 point2give[3];
  int res=0;
  double distConf, dist2Obj;
  configPt qIni =  p3d_get_robot_config(r);

  //double dist2point;
#ifdef HRI_JIDO
  int jointindexesR2[]= {5, 6, 7, 8, 9, 10, 13}; //Jido arm
  int jointArmBase = 16;
  int njoints = 7;
#elif defined HRI_HRP2
  int jointindexesR2[]= {14, 15, 19, 20, 21, 22, 23, 24, 29}; //HRP RIGHT HAND
  int njoints = 9;
  int jointArmBase = 19;
#elif defined HRI_TUM_BH
  int jointindexesR2[]=  {2,3,8,9,10,11,12}; //BH RIGHT ARM
  int njoints = 7;
  int jointArmBase = 9;
#elif defined HRI_BERT1
  int jointindexesR2[]= {14, 15, 19, 20, 21, 22, 23, 24, 29}; //HRP RIGHT HAND
  int njoints = 9;
  int jointArmBase = 19;
#else
#error Robot platform must be defined !!
#endif

  //find human receiving point
  //p3d_get_robot_center(obr, center);

  p3d_get_object_center(obj, center);

  if (r->joints[16]->o)
    p3d_get_object_center(r->joints[16]->o, pointIni);
  else
    p3d_get_robot_center(r, pointIni);

  center[2] = obj->BB.zmax + .1;

  ox = point2give[0][0] = center[0];
  oy = point2give[0][1] = center[1];
  oz = point2give[0][2] = center[2];
  //find a conf
  if (PSP_GIK2 != NULL)
  {
    printf("INITIALIZING ... \n");
    if(!PSP_GIK2->GIKInitialized){
      /***** FOR JIDO *****/
      hri_gik_initialize_gik(PSP_GIK2,r,1,njoints); //
      hri_gik_add_task(PSP_GIK2, 3, njoints, 1, jointindexesR2, ROBOTj_GRIP);  /* Placement */
    }
    //unsigned long msecSt,msecEnd;
    //int msecQual;
    //msecSt = ChronoGet();
    res = hri_gik_compute(r, PSP_GIK2, 200, 0.1, 1, 0, point2give, NULL, resq, NULL);
    //msecEnd = ChronoGet();
    // msecQual =(int)( msecEnd - msecSt);

    //printf("%d - %d == %i\n",msecSt,msecEnd,msecQual);

    if (res)
    {
      p3d_set_and_update_this_robot_conf(r,*resq);
      //find quality
      printf("PICK CONF. FOUND\n");
    }
    else
    {
      //resq = NULL;
      *quality = -1;
      printf("NO PICK CONF.FOUND\n");
      return(res);
    }

  }
  printf("Centers\n");
  if (r->joints[jointArmBase]->o)
    p3d_get_object_center(r->joints[jointArmBase]->o, point);
  else
    p3d_get_robot_center(r, point);

  printf("Distances .. \n");
  distConf =   p3d_dist_config(r, qIni,*resq);//Distance between original and final Configurations
                                              //printf("1 .. \n");
  distConf +=  DISTANCE3D(pointIni[0],pointIni[1],pointIni[2],point2give[0][0],point2give[0][1],point2give[0][2]);//Original Distance between finger(Object)  and object to reach
  dist2Obj =   DISTANCE3D(point[0],point[1],point[2],point2give[0][0],point2give[0][1],point2give[0][2]);//Final Distance between finger(Object)  and object to reach

  //printf("Distances \n     %f + %f == %f\n",distConf,dist2Obj, distConf+dist2Obj);


  if(dist2Obj<PSP_DIST2OBJ_TRSHLD)
    *quality = distConf+dist2Obj;
  else
    *quality = -1;

  return(res);

}


/****************************************************************/
/*!
 * \brief Finds a robot configuration to take an object at searchball position
 *
 * \param r - robot
 * \param obr - human
 * \param resq - resulting q
 * \param quality - Resultin messure of the feasability of the task
 * !
 */
/****************************************************************/

static int psp_take_it_at(p3d_rob* r, p3d_vector3 goalPoint, configPt *resq,  double *quality)
{
  p3d_vector4 point, pointIni;
  p3d_vector3 point2give[3];
  int res=0;
  double distConf, dist2Obj;
  configPt qIni =  p3d_get_robot_config(r);


  //double dist2point;
#ifdef HRI_JIDO
  int jointindexesR2[]= {5, 6, 7, 8, 9, 10, 13}; //Jido arm
  int jointArmBase = 16;
  int njoints = 7;
#elif defined HRI_HRP2
  int jointindexesR2[]= {14, 15, 19, 20, 21, 22, 23, 24, 29}; //HRP RIGHT HAND
  int njoints = 9;
  int jointArmBase = 19;
#elif defined HRI_TUM_BH
  int jointindexesR2[]=  {2,3,8,9,10,11,12}; //BH RIGHT ARM
  int njoints = 7;
  int jointArmBase = 9;
#elif defined HRI_BERT1
  int jointindexesR2[]= {14, 15, 19, 20, 21, 22, 23, 24, 29}; //HRP RIGHT HAND
  int njoints = 9;
  int jointArmBase = 19;
#else
#error Robot platform must be defined !!
#endif


  //find human receiving point
  //p3d_get_robot_center(obr, center);

  //p3d_get_object_center(obj, center);

  if (r->joints[16]->o)
    p3d_get_object_center(r->joints[16]->o, pointIni);
  else
    p3d_get_robot_center(r, pointIni);

  //center[2] = obj->BB.zmax + .1;

  printf("point to give %f, %f, %f\n", point2give[0][0],point2give[0][1],point2give[0][2]);//Final Distance between finger(Object)  and object to reach
  ox = point2give[0][0] = goalPoint[0];
  oy = point2give[0][1] = goalPoint[1];
  oz = point2give[0][2] = goalPoint[2];

  //find a conf
  if (PSP_GIK2 != NULL)
  {
    printf("INITIALIZING ... \n");
    if(!PSP_GIK2->GIKInitialized){
      /***** FOR JIDO *****/
      hri_gik_initialize_gik(PSP_GIK2,r,1,njoints); //
      hri_gik_add_task(PSP_GIK2, 3, njoints, 1, jointindexesR2, ROBOTj_GRIP);  /* Placement */
    }
    //unsigned long msecSt,msecEnd;
    //int msecQual;
    //msecSt = ChronoGet();
    res = hri_gik_compute(r, PSP_GIK2, 200, 0.1, 1, 0, point2give, NULL, resq, NULL);
    //msecEnd = ChronoGet();
    // msecQual =(int)( msecEnd - msecSt);

    //printf("%d - %d == %i\n",msecSt,msecEnd,msecQual);

    if (res)
    {
      p3d_set_and_update_this_robot_conf(r,*resq);
      //find quality
      printf("PICK CONF. FOUND\n");
    }
    else
    {
      //resq = NULL;
      *quality = -1;
      printf("NO PICK CONF.FOUND\n");
      return(res);
    }

  }
  printf("Centers\n");
  if (r->joints[16]->o)
    p3d_get_object_center(r->joints[jointArmBase]->o, point);
  else
    p3d_get_robot_center(r, point);
  printf("Distances .. \n");
  distConf =   p3d_dist_config(r, qIni,*resq);//Distance between original and final Configurations
                                              //printf("1 .. \n");
  distConf +=  DISTANCE3D(pointIni[0],pointIni[1],pointIni[2],goalPoint[0],goalPoint[1],goalPoint[2]);//Original Distance between finger(Object)  and object to reach
  dist2Obj =   DISTANCE3D(point[0],point[1],point[2],goalPoint[0],goalPoint[1],goalPoint[2]);//Final Distance between finger(Object)  and object to reach

  printf("Distances \n     %f + %f == %f\n",distConf,dist2Obj, distConf+dist2Obj);

  *quality = distConf+dist2Obj;

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

static double psp_test_qs(p3d_rob *r, configPt q1, configPt q2, p3d_vector4 point, double viewPercent , int checkTraj, hri_bitmapset* PSP_BTSET)
{
  int res=0;
  double qs[3], qg[3];
  double qzs=0;
  //  int kcd_with_report;
  double perspValue;


  //printf("------------------- Setting Robot---------------------\n");
  res = p3d_set_and_update_this_robot_conf(r,q1);
  //printf("------------------- Drawing windows---------------------\n");
  //g3d_draw_allwin_active();
  //g3d_refresh_allwin_active();
  ////movie stuff
  //g3d_save_movie_image();
  //g3d_refresh_allwin_active();

  if (res) // is a valid configuration?
  {
    res = 0;
    //printf("------------------- Turning head ---------------------\n");
    //if task is give object
    /* if (psp_give_to(r,obr, &q1,&giveValue))
     {
     printf("yes \n");
     }
     else
     {
     giveValue = 1.5;
     }*/
    //else
    // giveValue = 0;
    if (psp_look_at(r, point[0], point[1], point[2], &q1))//Can turn its head to center point?
                                                          //if (psp_set_pan_tilt(r,point,&q1))
    {
      ///uncomment this part below if gik doesn't test for collision
      // if (!checkTraj)
      //res = p3d_col_test_robot(r,kcd_with_report);
      //res = kcd_check_report(PSP_ROBOT->num); //This is the good one
      //else
      res =0;
      //g3d_refresh_allwin_active();
      if(!res) // isn't there a collision?  ------ p3d_col_test_choice(); p3d_col_env_set_traj_method(type); test_current_coll_call_times();
	    {
	      printf("------------------- Watching ---------------------\n");
	      ChronoOn();
	      perspValue = pso_watch3_obj();

	      ChronoPrint("TIME of perception");

	      ChronoOff();

	      if (isnan(perspValue))
          perspValue = 0.0;
	      if ( perspValue >= viewPercent)//pso_see_obj()) // can the robot see the object?
        {
          //printf("------------------- coping config ---------------------\n");
          p3d_copy_config_into(r,q1, &(r->ROBOT_GOTO));
          p3d_copy_config_into(r,q2, &(r->ROBOT_POS));
          res = 1;
          if (checkTraj)
          {
            //ChronoOn();
            res =0;
            if (DISTANCE2D(q1[ROBOTq_X],q1[ROBOTq_Y],q2[ROBOTq_X],q2[ROBOTq_Y])==0)
              res =1;
            else
            {
              //printf("------------------- checking Btset ---------------------\n");
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
#ifdef OFF_LINE
              qzs = r->ROBOT_POS[ROBOTq_Z];
#endif
              qs[2] = qzs;
              qg[0] = r->ROBOT_GOTO[ROBOTq_X];
              qg[1] = r->ROBOT_GOTO[ROBOTq_Y];
#ifdef OFF_LINE
              qzs = r->ROBOT_GOTO[ROBOTq_Z];
#endif
              qg[2] = qzs;

              hri_bt_reset_path(PSP_BTSET);

              res =  hri_bt_start_search(qs,qg, PSP_BTSET, 0);
              //printf("res: %i \n",res);
            }
            //ChronoPrint("TIME of path");

            //ChronoOff();

          }
          //glFlush();
          if (res>=0) // is there a valid path?

          {
            p3d_set_and_update_this_robot_conf(r,q1);
            printf("------------------- Modeling Point Found ---------------------\n");
            //p3d_destroy_config(r, q1);
            //p3d_destroy_config(r, q2);
            //g3d_refresh_allwin_active();
            //g3d_end_movie();

            return perspValue;
          }
          else
            printf("not valid path %i \n",res );

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
  return -1;
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
  return FALSE;
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
  //p3d_jnt *jntPt = objRob->joints[1];
  //double x,y,xo,yo,
  double zo;
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
  dist = linearDistance(qcurr[ROBOTq_X],qcurr[ROBOTq_Y],objqcurr[HUMANq_X], objqcurr[HUMANq_Y]);

  //if (p3d_is_in_pos_area(objRob,v_aux[0],v_aux[1],FALSE))
  if (dist<= objRob->max_pos_range && dist >= objRob->min_pos_range)
  {
    //printf("1 - in the area %f --> %f - %f = %f\n", dist, refHumAngle2,refHumAngle3, refHumAngle2-refHumAngle3 );
    if ( abs(refHumAngle3-refHumAngle2)<=M_PI/2)
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

	      if (zo < 0.65 && linearDistance(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y])<2)//check this conditional
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

	      qaux[ROBOTq_RZ] = angleLim(refHumAngle);
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

static int psp_set_vert2rconf(p3d_matrix4 matr,  psp_obs_vertex *vert , configPt qres)
{
  p3d_vector4 v_aux={0,0,0,1}, posPoint={0,0,0,1}, obPoint={0,0,0,1};
  //double rx,ry,rz;

  v_aux[0] = vert->pos[0];
  v_aux[1] = vert->pos[1];
  p3d_matvec4Mult( matr, v_aux,  posPoint );

  qres[ROBOTq_X]  = posPoint[0];
  qres[ROBOTq_Y]  = posPoint[1];

  //p3d_mat4ExtractPos(matr,&obPoint[0],&obPoint[1],&obPoint[2],&rx,&ry,&rz);
  //qres[ROBOTq_RZ] = angleLimit(rad_angleOf(posPoint[0],posPoint[1],obPoint[0],obPoint[1]));

  qres[ROBOTq_RZ] = angleLimit(rad_angleOf(posPoint[0],posPoint[1],ox,oy));

  return 1;
}


static double psp_check_vertex(p3d_rob* r,  p3d_matrix4 matr, double viewPercent , psp_obs_vertex *vert, configPt qcurr, configPt qres, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 v_aux={0,0,0,1}, posPoint={0,0,0,1}, obPoint={ox,oy,oz,1};
  double rx,ry,rz;
  p3d_copy_config_into(r,qcurr, &qres);
  if (vert->status == PSP_NON_OBSERVED)
  {
    psp_set_vert2rconf(matr,vert, qres);
    vert->quality =  psp_test_qs(r, qres, qcurr, obPoint, viewPercent, PSP_NHP_TRAJ, PSP_BTSET);

    testedQs++;
    if (vert->quality > 0)
    {
      vert->status  = PSP_OBSERVED;
      //			theqs[qindex]  =  p3d_copy_config(r,qres);
      //			qindex++;
      //printf("good conf\n");
    }
    else
      vert->status  = PSP_NOT_AVAILABLE;

    return vert->quality;
  }
  return -1;//not available
}


static double psp_check_vertexL(p3d_rob* r,  p3d_matrix4 matr, double viewPercent , psp_obs_vertex *vert, configPt qcurr, configPt qres, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 v_aux={0,0,0,1}, posPoint={0,0,0,1}, obPoint={ox,oy,oz,1};
  double rx,ry,rz;
  p3d_copy_config_into(r,qcurr, &qres);
  if (vert->status == PSP_St_OBSERVABLE)
  {
    psp_set_vert2rconf(matr,vert, qres);
    vert->quality =  psp_test_qs(r, qres, qcurr, obPoint, viewPercent, PSP_NHP_TRAJ, PSP_BTSET);

    testedQs++;
    if (vert->quality > 0)
    {
      vert->status  = PSP_St_OBSERVED;
      //			theqs[qindex]  =  p3d_copy_config(r,qres);
      //			qindex++;
      //			printf("good conf\n");
    }
    else
      vert->status  = PSP_St_NOT_IN_RANGE;

    return vert->quality;
  }
  return -1;//not available
}


int psp_get_evaluate_task_goal (p3d_rob* r, configPt qcurr, configPt qres, psp_obs_vertex *vert)
{
  double taskQual, taskCost;
  int resTask = task_eval(r,taskTarget, &qres, &taskQual, &taskCost);
  double taskGain = 1.0 - PSP_DIST2OBJ_TRSHLD;
  double taskutility =
  psp_get_point_utility(
                        (((pspTaksMaxQual-taskQual)*100)/pspTaksMaxQual),
                        ((taskCost*100)/pspTaksMaxCost));

  if (resTask==0)
  {
    //printf("NO %f\n",taskutility);
    taskutility*=0.1;
  }
  if (resTask<0)
  {
    //printf("NO %f\n",taskutility);
    taskutility=-3.0;
  }

  vert->utility =  (vert->utility*taskGain)+(taskutility*PSP_DIST2OBJ_TRSHLD);  //(vert->utility+taskutility)/2.0;
                                                                                //vert.utility = utiltmp;
                                                                                //printf("UtilTask = %f\n",utiltmp);

  return resTask;
}

static int psp_get_best_neig(p3d_rob* r,  p3d_matrix4 matr, double viewPercent , psp_lst_vertex *lstVtx, configPt qcurr, configPt qres, hri_bitmapset* PSP_BTSET)
{
  //p3d_vector4 obPoint
  int choix = lstVtx->chosen;
  double quality, utilMax = lstVtx->vertex[choix].utility;
  psp_obs_vertex *vertG, *vertL;
  int seg = lstVtx->vertex[choix].segment;
  int lay = lstVtx->vertex[choix].layer;
  int found = 0, resTask;
  if (seg > 0)
  {
    vertG = &lstVtx->grid[seg-1][lay];
    quality = psp_check_vertex(r, matr, viewPercent, vertG, qcurr,qres, PSP_BTSET);
    vertL = &lstVtx->vertex[vertG->id];
    if (quality > 0)
    {
      vertL->status = PSP_St_OBSERVED;
      vertL->quality = quality;
      vertG->utility = vertL->utility = psp_get_point_utility(quality,vertL->cost);
      if (task_eval)//PSP_NEXT_TASK != PSP_NO_TASK)
	    {
	      resTask = psp_get_evaluate_task_goal(r, qcurr, qres, vertL);
	    }
      else
        resTask =1;

      theqs[qindex]  =  p3d_copy_config(r,qres);
      qindex++;

      if (resTask>=0)
	    {
	      if( vertL->utility > utilMax )
        {
          utilMax = vertL->utility;
          lstVtx->chosen = vertG->id;
          //besti = qindex-1;
          found = 1;
        }
	    }
      else
	    {
	      vertL->status = PSP_St_NOT_IN_RANGE;
	      vertG->status = PSP_NOT_AVAILABLE;
	    }
    }
    else
      vertL->status = PSP_St_NOT_IN_RANGE;
  }

  if (seg <= lstVtx->ns)
  {
    vertG = &lstVtx->grid[seg+1][lay];
    quality = psp_check_vertex(r, matr, viewPercent, vertG, qcurr,qres, PSP_BTSET);
    vertL = &lstVtx->vertex[vertG->id];
    if (quality > 0)
    {
      vertL->status = PSP_St_OBSERVED;
      vertL->quality = quality;
      vertG->utility = vertL->utility = psp_get_point_utility(quality,vertL->cost);
      if (task_eval)//PSP_NEXT_TASK != PSP_NO_TASK)
	    {
	      resTask = psp_get_evaluate_task_goal(r, qcurr, qres, vertL);
	    }
      else
        resTask =1;
      theqs[qindex]  =  p3d_copy_config(r,qres);
      qindex++;
      if (resTask>=0)
	    {
	      if( vertL->utility > utilMax )
        {
          utilMax = vertL->utility;
          lstVtx->chosen = vertG->id;
          //besti = qindex-1;
          found = 1;
        }
	    }
      else
	    {
	      vertL->status = PSP_St_NOT_IN_RANGE;
	      vertG->status = PSP_NOT_AVAILABLE;
	    }
    }
    else
      vertL->status = PSP_St_NOT_IN_RANGE;

  }

  if (lay > 0)
  {
    vertG = &lstVtx->grid[seg][lay-1];
    quality = psp_check_vertex(r, matr, viewPercent, vertG, qcurr,qres, PSP_BTSET);
    vertL = &lstVtx->vertex[vertG->id];
    if (quality > 0)
    {
      vertL->status = PSP_St_OBSERVED;
      vertL->quality = quality;
      vertG->utility = vertL->utility = psp_get_point_utility(quality,vertL->cost);
      if (task_eval)//PSP_NEXT_TASK != PSP_NO_TASK)
	    {
	      resTask = psp_get_evaluate_task_goal(r, qcurr, qres, vertL);
	    }
      else
        resTask =1;
      theqs[qindex]  =  p3d_copy_config(r,qres);
      qindex++;
      if (resTask>=0)
	    {
	      if( vertL->utility > utilMax )
        {
          utilMax = vertL->utility;
          lstVtx->chosen = vertG->id;
          //besti = qindex-1;
          found = 1;
        }
	    }
      else
	    {
	      vertL->status = PSP_St_NOT_IN_RANGE;
	      vertG->status = PSP_NOT_AVAILABLE;
	    }

    }
    else
      vertL->status = PSP_St_NOT_IN_RANGE;
  }

  if (lay < lstVtx->nl)
  {
    vertG = &lstVtx->grid[seg][lay+1];
    quality = psp_check_vertex(r, matr, viewPercent, vertG, qcurr,qres, PSP_BTSET);
    vertL = &lstVtx->vertex[vertG->id];
    if (quality > 0)
    {
      vertL->status = PSP_St_OBSERVED;
      vertL->quality = quality;
      vertG->utility = vertL->utility = psp_get_point_utility(quality,vertL->cost);
      if (task_eval)//PSP_NEXT_TASK != PSP_NO_TASK)
	    {
	      resTask = psp_get_evaluate_task_goal(r, qcurr, qres, vertL);
	    }
      else
        resTask =1;
      theqs[qindex]  =  p3d_copy_config(r,qres);
      qindex++;
      if (resTask>=0)
	    {
	      if( vertL->utility > utilMax )
        {
          utilMax = vertL->utility;
          lstVtx->chosen = vertG->id;
          //besti = qindex-1;
          found = 1;
        }
	    }
      else
	    {
	      vertL->status = PSP_St_NOT_IN_RANGE;
	      vertG->status = PSP_NOT_AVAILABLE;
	    }

    }
    else
      vertL->status = PSP_St_NOT_IN_RANGE;
  }
  return found;
}

static int psp_local_grad_max (p3d_rob* r,  p3d_matrix4 matr, double viewPercent , psp_lst_vertex *lstVtx, configPt qcurr, configPt qres, hri_bitmapset* PSP_BTSET)
{
  psp_obs_vertex *vert;
  //store chosen utility
  int choix = lstVtx->chosen;
  //int choix = lstVtx->currentVert;
  double maxUtil = lstVtx->vertex[choix].utility;
  vert = &lstVtx->vertex[choix];
  vert->status =   PSP_St_OBSERVED;
  lstVtx->grid[vert->segment][vert->layer].status = PSP_OBSERVED;

  //printf("Searching neighbors \n");
  //start search it's neighbors
  if (psp_get_best_neig(r, matr, viewPercent, lstVtx, qcurr, qres, PSP_BTSET))
    return psp_local_grad_max (r, matr, viewPercent, lstVtx, qcurr, qres, PSP_BTSET);
  else
  {
    //lstVtx->chosen = lstVtx->currentVert;
    printf("Best Gradient %i \n",choix);
    /*choix = lstVtx->chosen;
     printf("---Best Gradient %i \n",choix);
     vert = &lstVtx->vertex[choix];
     p3d_vector4 v_aux={0,0,0,1}, posPoint={0,0,0,1}, obPoint={0,0,0,1};
     double rx,ry,rz;

     v_aux[0] = vert->pos[0];
     v_aux[1] = vert->pos[1];
     p3d_matvec4Mult( matr, v_aux,  posPoint );

     qres[ROBOTq_X]  = posPoint[0];
     qres[ROBOTq_Y]  = posPoint[1];

     p3d_mat4ExtractPos(matr,&obPoint[0],&obPoint[1],&obPoint[2],&rx,&ry,&rz);

     qres[ROBOTq_RZ] = angleLimit(rad_angleOf(posPoint[0],posPoint[1],obPoint[0],obPoint[1]));
     p3d_copy_config_into(r,qres, &(r->ROBOT_GOTO));*/
    //p3d_set_and_update_this_robot_conf(r, qcurr);
    return 1;
  }
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



int psp_srch_model_pt(p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method , double viewPercent, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point, point2, v_aux;
  p3d_jnt *jntPt = objRob->joints[1];

  //time messure variables
  clock_t start,end;
  float time_psp;
  ///////
  //double refHumAngle, refHumAngle2, refHumAngle3;
  double xo,yo,zo;

  configPt qcurr, qaux,objqcurr, taskqcurr;
  int i, res, *vIndex,resTask=-1;
  int besti, response;
  int randFlag = 0;
  int currVert, lasttime;
  double qqual, *Qquals, maxcost;
  double taskCost,maxTaskQual, maxTaskCost;
  p3d_matrix4 rotMat = {{1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}};
  //double kcd_with_report;

  //FILE * ftaskCost =  fopen("taskcost.dat","w");
  int endLimit;
  //Qquals = malloc(sizeof(float)*((numpoints+2)*numlayers));
  int allocsize = ((numpoints+2)*numlayers);
  Qquals = MY_ALLOC(double,allocsize);
  vIndex = (int*) malloc(sizeof(int)*((numpoints+2)*numlayers));
  //testedX = malloc(sizeof(float)*((numpoints+2)*numlayers));
  //testedY = malloc(sizeof(float)*((numpoints+2)*numlayers));

  ChronoOn();
  start = clock();
  PSP_DRAW_QS = FALSE;
  // double f_min, f_max;
  //  p3d_jnt *jntPt2;
  //  int i_DoF;

  //jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_RZ+1,&i_DoF);
  //p3d_jnt_get_dof_bounds(jntPt2,i_DoF, &f_min, &f_max);

  destIndex = -1;
  qcurr     = p3d_get_robot_config(r);
  qaux      = p3d_get_robot_config(r);
  taskqcurr = p3d_get_robot_config(r);
  objqcurr  = p3d_get_robot_config(objRob);
  if (qcurr == NULL || qaux == NULL ||  objqcurr == NULL)
  {
    printf ("Impossible to allocate memory\n");
    return 0;
  }
  point[0] = qaux[ROBOTq_X];
  point[1] = qaux[ROBOTq_Y];
  point[2] = 0.0;
  point[3] = 1.0;

  v_aux[0] = qaux[ROBOTq_X] - objqcurr[HUMANq_X];
  v_aux[1] = qaux[ROBOTq_Y] - objqcurr[HUMANq_Y];
  v_aux[2] = 0.0;


  point[0] = 1.0;
  point[1] = 0.0;
  point[2] = 0.0;
  point[3] = 1.0;

  // p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);
  //
  //   refHumAngle2 = rad_angleOf(objqcurr[ROBOTq_X],objqcurr[ROBOTq_Y],v_aux[0],v_aux[1]);


  //   printf(" Human Angle %f\n",refHumAngle2);
  point[0] = 0.0;

  p3d_get_robot_center(objRob, point2);
  ///just in case that we want to extract the position from the matrix abs. or  from the BB
  //p3d_mat4ExtractPos(jntPt->abs_pos,&v_aux[0],&v_aux[1],&v_aux[2],&xo,&yo,&zo);
  //printf("point compare (%f %f %f) = (%f %f %f) \n",point2[0],point2[1],point2[2], v_aux[0],v_aux[1],v_aux[2]);


  rotMat[0][3] = ox = xo = point2[0];
  rotMat[1][3] = oy = yo = point2[1];
  zo = point2[2];
  //printf("x -> %f , %f\n",x,xo);
  //printf("y -> %f , %f\n",y,yo);
  //sphereActive =1;
  if (search_method[PSP_SRCHM_METHOD] == PSP_FRONT  && psp_is_a_human(objRob))
    oz = point2[2] = zo + (objRob->joints[55]->abs_pos[2][3]-zo)/1.5;
  else
    oz = point2[2];
  printf("z -> %f \n",zo);


  point[0] = 0.0;
  point[1] = 0.0;
  point[2] = 0.0;
  point[3] = 1.0;

  v_aux[0] = 0.0;
  v_aux[1] = 0.0;
  v_aux[2] = 0.0;

  qindex   = 0;
  testedQs = 0;

  //lastAngleCam  = qaux[ROBOTq_PAN];
  ChronoOn();
  InitWaveCells(r->env->box.x1,r->env->box.y1,r->env->box.x2,r->env->box.y2, qaux[ROBOTq_X], qaux[ROBOTq_Y],PSP_BTSET);
  ChronoPrint("Wave time:");
  ChronoOff();
  //printGridVals();
  //printWaveVals();
  //printObstacles();
  //printGridObst();

  //jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_PAN,&i_DoF);
  psp_gen_points(search_method[PSP_SRCHM_METHOD], objRob, r, numpoints, numlayers, &lstvert, PSP_BTSET);
  //printf("------------------------>>>>>>>> dist min %f \n", linearDistance(lstvert.grid[2][0].pos[0], lstvert.grid[2][0].pos[1], lstvert.grid[3][0].pos[0], lstvert.grid[3][0].pos[1]));

  //printf("------------------------>>>>>>>> dist max%f \n", linearDistance(lstvert.grid[2][lstvert.nl-1].pos[0], lstvert.grid[2][lstvert.nl-1].pos[1], lstvert.grid[3][lstvert.nl-1].pos[0], lstvert.grid[3][lstvert.nl-1].pos[1]));

  printf("List generated\n");
  psu_reboot_theqs(r,lstvert.nv );
  //theqs = realloc(theqs,sizeof(configPt*)*lstvert.nv);
  /*
   if (qindAnt<lstvert.nv)
   {
   for (i=qindAnt;i<lstvert.nv;i++)
   {
   theqs = p3d_get_robot_config(r);
   }
   }
   */

  printf("memory allocated\n");

  //printListVtx(&lstvert);
  if (search_method[PSP_SRCHM_TYPE] == PSP_ORDERED)
	{
	  printf("Search Type: ORDERED\n");
	  psp_order_point_list(&lstvert);
	  printf("List Ordered\n");
	}

  if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM ||search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
	{
	  srand((unsigned)time(0));
	  randFlag = TRUE;

	}

  if (PSP_NEXT_TASK != PSP_NO_TASK)
	{
	  p3d_copy_config_into(r,qaux,&taskqcurr);
	  if (psp_is_a_human(objRob))
    {
      task_eval = psp_give_to;
      resTask   = task_eval(r, objRob, &taskqcurr, &maxTaskQual, &maxTaskCost);
      pspTaksMaxCost = maxTaskCost;
      pspTaksMaxQual = maxTaskQual;
      taskTarget = objRob;
      sphereActive =1;

    }
	  else//TODO:put this task_eval also for this task_eval = psp_take_from_surface;
	    resTask = psp_take_from_surface(r, objRob->o[0], &taskqcurr, &maxTaskQual);
	}
  else
    task_eval = NULL;


  ///On the first configuration
	p3d_set_and_update_this_robot_conf(r,qcurr);


	if (search_method[PSP_SRCHM_GOAL] ==  PSP_DEEP || randFlag)
	  endLimit = PSP_STOP_DEEP;
	else
	  endLimit = lstvert.nv;




	lasttime = 0;
	for (i=0;i<endLimit && !lasttime;i++)
  {

    if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM)
    {
      // res = psp_get_next_random_point(point, numlayers, numpoints, &lstvert);
      res = psp_get_next_random_point_list(point, numlayers, numpoints, &lstvert);
      currVert = lstvert.currentVert;
    }
    else
    {
      if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
		  {
		    res = psp_get_next_random_point_list(point, numlayers, numpoints, &lstvert);
		    currVert = lstvert.currentVert;
		  }
      else
		  {
		    res = psp_get_next_ordered_point(point,&lstvert);
		    currVert = lstvert.currentVert-1;
		    if (currVert == -1)
        {
          currVert = (lstvert.nv - 1);
          lasttime =  TRUE;
        }
		  }
    }



    if(res)
    {
      printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
      // passing from local human  to global coords
      if (search_method[PSP_SRCHM_METHOD] == PSP_FRONT)
		  {
		    //p3d_matvec4Mult(jntPt->abs_pos,point,v_aux
		    //psp_set_vert2rconf(jntPt->abs_pos,&lstvert.vertex[currVert], qaux);
		    qqual = psp_check_vertexL(r, jntPt->abs_pos, viewPercent, &lstvert.vertex[currVert], qcurr,qaux, PSP_BTSET);
		  }
      else
		  {
		    //v_aux[0] = point[0] +  point2[0];
		    //v_aux[1] = point[1] +  point2[1];
		    //psp_set_vert2rconf(rotMat,&lstvert.vertex[currVert], qaux);
		    qqual = psp_check_vertexL(r, rotMat, viewPercent, &lstvert.vertex[currVert], qcurr,qaux, PSP_BTSET);
		  }


      if(qqual > -1)
		  {

		    lstvert.vertex[ currVert ].quality = qqual;
		    lstvert.grid[lstvert.vertex[ currVert ].segment][lstvert.vertex[currVert].layer].quality = qqual;
		    vIndex[qindex] = currVert;
		    ///setting the utility of a point
        double utiltmp = psp_get_point_utility(qqual, lstvert.vertex[ currVert ].cost);
        lstvert.vertex[ currVert ].utility = utiltmp;
        lstvert.grid[lstvert.vertex[currVert].segment][lstvert.vertex[currVert].layer].utility = utiltmp;
        lstvert.vertex[ currVert ].status = PSP_St_OBSERVED;


        ///if task is give object
        double taskValue,taskutility;
        double taskGain = 1.0 - PSP_DIST2OBJ_TRSHLD;
        if (task_eval)//PSP_NEXT_TASK != PSP_NO_TASK)
			  {

			    p3d_copy_config_into(r,qaux,&taskqcurr);
			    //sphereActive =1;
			    resTask = task_eval(r,taskTarget, &taskqcurr,&taskValue,&taskCost);

			    // 				if (psp_is_a_human(objRob))
			    // 					resTask = psp_give_to(r,objRob, &taskqcurr,&taskValue,&taskCost);
			    // 				else
			    // 					resTask = psp_take_from_surface(r,objRob->o[0], &taskqcurr,&taskValue);//TODO:modify this function to also give cost

			    taskutility = psp_get_point_utility((((maxTaskQual-taskValue)*100)/maxTaskQual),((taskCost*100)/maxTaskCost));

			    if (resTask==0)
          {
            //printf("NO %f\n",taskutility);
            taskutility*=0.7;//
          }
			    if (resTask<0)
          {
            //printf("NO %f\n",taskutility);
            taskutility*=0.1;
          }
			    //else
			    utiltmp = (utiltmp*taskGain)+(taskutility*PSP_DIST2OBJ_TRSHLD);
			    lstvert.vertex[ currVert ].utility =  utiltmp;
			    lstvert.grid[lstvert.vertex[currVert].segment][lstvert.vertex[currVert].layer].utility = utiltmp;
			    //printf("UtilTask = %f\n",utiltmp);
			  }
        else
          resTask =1;


        if (search_method[PSP_SRCHM_GOAL] ==  PSP_FFFO)
			  {
			    if (resTask<0)
			      continue;

			    if (resTask)
          {
            p3d_set_and_update_this_robot_conf(r,qcurr);
            p3d_destroy_config(objRob,objqcurr);
            g3d_draw_allwin_active();
            //PSP_DRAW_QS = TRUE;
            ChronoPrint("PSP - TIME");
            ChronoOff();
            //printListVtx(&lstvert);
            destIndex = currVert;
            printf("---- %i Configurations  Generated -----\n",lstvert.nv);
            printf("---- %i Configurations  Tested    -----\n",testedQs);
            printf("---- Best configuration  on %i with %f\n",destIndex, utiltmp);
            p3d_destroy_config(r,qaux);
            p3d_destroy_config(r,qcurr);
            p3d_destroy_config(r,taskqcurr);
            free(Qquals);
            free(vIndex);
            return TRUE;
          }
			  }
        else
			  {
			    //theqs[qindex]  = realloc(theqs[qindex], sizeof());
			    ///TODO:change this for the other two: May not be necessary
			    //p3d_copy_config_into(r,qaux, &theqs[qindex]);

			    if (PSP_NEXT_TASK != PSP_NO_TASK)
			      theqs[qindex]  =  p3d_copy_config(r,taskqcurr);
			    else
			      theqs[qindex]  =  p3d_copy_config(r,qaux);


			    Qquals[qindex] = qqual;

			    if (qindex == 0)
          {

            if ((search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST) || (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM))
            {
              lstvert.chosen = currVert;
              qindex++;
              if (search_method[PSP_SRCHM_METHOD] != PSP_FRONT)
				      {
                if(psp_local_grad_max(r, rotMat, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                {
                  besti     =  qindex-1;//currVert;//lstvert.chosen;
                  maxcost   =  lstvert.vertex[lstvert.chosen].utility;
                  destIndex =  lstvert.chosen;
                }
				      }
              else
				      {
                if(psp_local_grad_max(r, jntPt->abs_pos, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                {
                  besti     =  qindex-1;//currVert;//lstvert.chosen;
                  maxcost   =  lstvert.vertex[lstvert.chosen].utility;
                  destIndex =  lstvert.chosen;
                }
				      }
              qindex--;

            }
            else
            {
              besti = qindex;
              maxcost = utiltmp;
              //bestTaskVal = taskValue;
              destIndex = currVert;
            }
          }
			    else
          {
            int localbesti  = qindex;
            int localChosen = currVert;

            if(search_method[PSP_SRCHM_TYPE] == PSP_RANDOM)
            {
              lstvert.chosen = currVert;
              qindex++;
              if (search_method[PSP_SRCHM_METHOD] != PSP_FRONT)
				      {
                if(psp_local_grad_max(r, rotMat, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                {
                  localbesti     =  qindex-1;//currVert;//lstvert.chosen;
                  utiltmp        =  lstvert.vertex[lstvert.chosen].utility;
                  localChosen    =  lstvert.chosen;
                }
				      }
              else
				      {
                if(psp_local_grad_max(r, jntPt->abs_pos, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                {
                  localbesti     =  qindex-1;//currVert;//lstvert.chosen;
                  utiltmp        =  lstvert.vertex[lstvert.chosen].utility;
                  localChosen    =  lstvert.chosen;
                }
				      }
              qindex--;
            }

            if(utiltmp>maxcost)
            {
              if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
				      {
                lstvert.chosen = currVert;
                qindex++;
                if (search_method[PSP_SRCHM_METHOD] != PSP_FRONT)
                {
                  if(psp_local_grad_max(r, rotMat, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                  {
                    besti     =  qindex-1;//currVert;//lstvert.chosen;
                    maxcost   =  lstvert.vertex[lstvert.chosen].utility;
                    destIndex =  lstvert.chosen;
                    endLimit  = 0;
                  }
                }
                else
                {
                  if(psp_local_grad_max(r, jntPt->abs_pos, viewPercent , &lstvert, qcurr, qaux, PSP_BTSET))
                  {
                    besti     =  qindex-1;//currVert;//lstvert.chosen;
                    maxcost   =  lstvert.vertex[lstvert.chosen].utility;
                    destIndex =  lstvert.chosen;
                    endLimit  = 0;
                  }
                }
                qindex--;
				      }
              else
				      {
                besti     = localbesti;// qindex;
                maxcost   = utiltmp;
                destIndex = localChosen; //currVert;
                                         //bestTaskVal = taskValue;
				      }
            }
          }
			    qindex++;
			  }//end else  search_method
         // 		   fprintf(ftaskCost,"%f %f %f \n",x,y,taskValue);
		  }
      else
		  {
		    if (qindex==0 && ((i+1)<lstvert.ns) && ((i+1)>=endLimit))
		      endLimit++;
		    lstvert.vertex[currVert].status =  PSP_St_HIDDEN;
		  }
    }
    // else
    //	printf("Point %i out of range\n",i);
  }

	p3d_set_and_update_this_robot_conf(r,qcurr);

	if (qindex==0)
  {
    printf("---- ERROR: Not modeling point found -----\n");
    printf("---- %i Configurations  Tested    -----\n",testedQs);
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
    response = FALSE;
  }
	else
  {
    printf("---- Matrix of  %i x %i -----\n",lstvert.ns, lstvert.nl);
    printf("---- %i Configurations  Generated -----\n",lstvert.nv);
    printf("---- %i Configurations  Tested    -----\n",testedQs);
    printf("---- %i Configurations  Found     -----\n",qindex);

    end = clock();
    time_psp = (end - start) * 1e-6;
    //printQquals(vIndex,Qquals,qindex,&lstvert);
    //p3d_set_and_update_this_robot_conf(r,theqs[besti]);
    point[0] = theqs[besti][ROBOTq_X];
    point[1] = theqs[besti][ROBOTq_Y];
    point[2] = 0.0;
    point[3] = 1.0;
    //destIndex = besti;
    if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM || search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
    {
      printf("---- Local grad -----\n ");
      if (search_method[PSP_SRCHM_METHOD] != PSP_FRONT)
        psp_set_vert2rconf(rotMat,&lstvert.vertex[destIndex], qaux);
      else
        psp_set_vert2rconf(jntPt->abs_pos,&lstvert.vertex[destIndex], qaux);
      printf("---- Best configuration  on %i with %f in %f %f  -----\n", destIndex, maxcost, qaux[ROBOTq_X], qaux[ROBOTq_Y]);
      if (pspResultsFile)
		  {
		    fprintf(pspResultsFile, "%i %i %i %f %f %f %f\n",lstvert.nv, testedQs, qindex, time_psp, maxcost,  qaux[ROBOTq_X], qaux[ROBOTq_Y]);
		  }
      //if (maxcost==1.58)
      if (maxcost>=MaxUtil_Res)
        PSP_RES_counter_max++;

      PSP_RES_avg_tested += testedQs;
      PSP_RES_avg_found += qindex;
      PSP_RES_avg_time += time_psp;
      PSP_RES_avg_utility += maxcost;
      p3d_copy_config_into(r,qaux, &(r->ROBOT_GOTO));
    }
    else
    {
      printf("---- Best configuration  on %i with %f in %f %f  -----\n", destIndex, maxcost, theqs[besti][ROBOTq_X], theqs[besti][ROBOTq_Y]);
      if (pspResultsFile)
		  {
		    fprintf(pspResultsFile, "%i %i %i %f %f %f %f\n",lstvert.nv, testedQs, qindex, time_psp, maxcost,  theqs[besti][ROBOTq_X], theqs[besti][ROBOTq_Y]);
		  }
      MaxUtil_Res = maxcost;
      p3d_copy_config_into(r,theqs[besti], &(r->ROBOT_GOTO));
    }

    printf("---- finnish -----\n ");


    //p3d_set_and_update_this_robot_conf(r,qcurr);

    //PSP_DRAW_QS = TRUE;
    // for (i=0;i<testedQs;i++)
    //{
    //printf("%f %f\n",theqs[i][ROBOTq_X],theqs[i][ROBOTq_Y]);
    //	  printf("%f %f\n",testedX[i],testedY[i]);
    //	  }
    printListVtx(&lstvert);
    response = TRUE;

  }
	//   fclose(ftaskCost);
	//printf("freeing...\n");
	free(Qquals);
	free(vIndex);
	//printf("freeing 2...\n");
	//free(testedX);
	//free(testedY);
	p3d_destroy_config(r,qaux);
	p3d_destroy_config(r,qcurr);
	p3d_destroy_config(objRob,objqcurr);
	//printf("all free...\n");
	//g3d_refresh_allwin_active();
	task_eval = NULL;
	printf(" --- CPU PSP - TIME %f \n", time_psp);
	ChronoPrint("PSP - TIME");
	ChronoOff();
	//g3d_end_movie();
	//PSP_DRAW_QS = TRUE;
	return response;
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


int psp_srch_model_pt_obj(p3d_rob* r, p3d_obj* object, int numsegs, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET)//firsttime and surface and with looking or not
{
  p3d_vector4 point,objCenter;
  double refObjAngle;
  double x,y,xo,yo,zo;
  configPt qcurr, qaux, taskqcurr;
  int i,resTask;
  psp_lst_vertex *lstvertex;
  float qQual, *QQuals, maxQual;
  int  besti, response;
  //  double kcd_with_report=0;
  double taskValue,bestTaskVal;

  QQuals = (float*) malloc(sizeof(float)*(numsegs+2)*numlayers);
  //vIndex = malloc(sizeof(float)*numsegs*numlayers);
  PSP_DRAW_QS = FALSE;

  lstvertex=&lstvert;
  p3d_get_object_center(object, objCenter);
  globaljnt = FALSE;
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r);
  taskqcurr = p3d_get_robot_config(r);
  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  destIndex = -1;
  ChronoOn();
  //lstvertex =  MY_ALLOC(psp_lst_vertex_tmp,1);
  //lstvertex =  MY_ALLOC(psp_lst_vertex,1);
  //if (!lstvertex)
  //  printf("NON alloc 1\n");
  // lstvertex->vertex = MY_ALLOC(psp_obs_vertex,numpoints);
  //if (!lstvertex->vertex)
  //  printf("NON alloc 2\n");
  InitWaveCells(r->env->box.x1,r->env->box.y1,r->env->box.x2,r->env->box.y2, qaux[ROBOTq_X], qaux[ROBOTq_Y],PSP_BTSET);
  //printGridVals();
  // printWaveVals();
  // printGridObst();
  //printCombinedBitmap();
  //psp_init_lst_vertex_obj(&lstvertex, numpoints , 1);
  //printGridVals();
  //printCombinedBitmap();
  //printObstacles();
  psp_gen_point_list_obj_complete (object, r, numsegs, numlayers,lstvertex, PSP_BTSET);
  ox = xo = objCenter[0];
  oy = yo = objCenter[1];
  oz = zo = objCenter[2];
  printf("...... on center point %f, %f,%f\n",ox,oy,oz);
  theqs = (double**) realloc(theqs,sizeof(configPt*)*lstvert.nv);

  if (search_method[PSP_SRCHM_TYPE] == PSP_ORDERED)
  {
    printf("Search Type: ORDERED\n");
    psp_order_point_list(lstvertex);
    printf("List Ordered\n");
  }
  if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM)
    srand((unsigned)time(0));
  qindex=0;
  testedQs=0;
  for (i=0;i<lstvertex->nv;i++)
  {

    if(psp_get_next_ordered_point(point,lstvertex))
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


	    //printf("x -> %f , %f\n",x,xo);
	    //printf("y -> %f , %f\n",y,yo);
	    //printf("z -> %f \n",zo);

	    ///Horizontal robot angle
      refObjAngle = rad_angleOf(x,y,xo,yo);

      //printf("Angle  %f", refObjAngle);
      qaux[ROBOTq_RZ] = angleLim(refObjAngle);

      qQual = psp_test_qs(r, qaux, qcurr,objCenter,viewPercent,1, PSP_BTSET);
      testedQs++;

      if(qQual >-1)
      {
        if (PSP_NEXT_TASK != PSP_NO_TASK)
		    {

		      p3d_copy_config_into(r,qaux,&taskqcurr);
		      //sphereActive =1;
		      //printf("Searching for task...\n");
		      if ((resTask=psp_take_from_surface(r,object, &taskqcurr,&taskValue)))
          {
            printf("YES %f\n",taskValue);

          }
		      else
          {
            printf("NO %f\n",taskValue);
          }
		    }
        else
		    {
		      printf("NO NEXT TASK\n");
		      taskValue = 0;
		      resTask = 1;
		    }

        if (taskValue>-1 && resTask)
		    {
		      //if(!p3d_col_test_robot(r,kcd_with_report))
		      if (search_method[PSP_SRCHM_GOAL] ==  PSP_FFFO)
          {
            g3d_refresh_allwin_active();
            theqs[qindex]  =  p3d_copy_config(r,qaux);
            qindex++;
            p3d_destroy_config(r,qaux);
            p3d_destroy_config(r,qcurr);
            destIndex = i;
            free(QQuals);
            ChronoPrint("PSP - TIME");
            ChronoOff();
            return TRUE;
          }
		      else
          {
            printf("Storing Conf. %i\n",i);
            //theqs[qindex]  =  p3d_copy_config(r,qaux);
            if (PSP_NEXT_TASK != PSP_NO_TASK)
              theqs[qindex]  =  p3d_copy_config(r,taskqcurr);
            else
              theqs[qindex]  =  p3d_copy_config(r,qaux);

            //printf("Stored\n");
            QQuals[qindex] =  qQual;
            printf("Stored Quality list\n");
            if (qindex == 0)
            {
              besti = qindex;
              maxQual = qQual;
              bestTaskVal = taskValue;
            }
            else
            {
              if (PSP_NEXT_TASK != PSP_NO_TASK)
              {
                if (abs(qQual-maxQual)<5 && taskValue<bestTaskVal)
                {
                  besti   = qindex;
                  maxQual = qQual;
                  destIndex = i;
                  bestTaskVal = taskValue;
                }
              }
              else
              {
                if (qQual>maxQual)
                {
                  besti   = qindex;
                  maxQual = qQual;
                  destIndex = i;
                  //bestTaskVal = taskValue;
                }
              }
            }
            qindex++;
          }
		    }
        else
          printf("Not feasible task in %i \n",i);
      }


    }
    else
      printf("Point %i out of range\n",i);
  }//end for

  p3d_set_and_update_this_robot_conf(r,qcurr);

  if (qindex==0)
  {
    printf("---- ERROR: Not modeling point found -----\n");
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
    response = FALSE;
  }

  else
  {
    //p3d_set_and_update_this_robot_conf(r,qcurr);
    printf("---- %i Configurations  Generated -----\n",lstvert.nv);
    printf("---- %i Configurations  Tested    -----\n",testedQs);
    printf("---- %i Configurations  Found     -----\n",qindex);
    printf("---- Best configuration  on %i with %f in %f %f  -----\n", besti, maxQual,theqs[besti][ROBOTq_X],theqs[besti][ROBOTq_Y]);
    //p3d_set_and_update_this_robot_conf(r,theqs[besti]);
    p3d_copy_config_into(r,theqs[besti], &(r->ROBOT_GOTO));
    //destIndex = besti;
    response = TRUE;
  }

  p3d_destroy_config(r,qaux);
  p3d_destroy_config(r,qcurr);
  g3d_refresh_allwin_active();
  ChronoPrint("PSP - TIME");
  ChronoOff();
  return response;
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
  //  double refObjAngle;
  double x,y,z,xo,yo,zo, zcenter=0.0;
  configPt qcurr, qaux;
  int i;
  // psp_lst_vertex lstvert2;


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
            //if (pso_watch2_obj()>=80.0)// can the robot see the object?
            if (pso_see_obj())
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
  while(psp_srch_model_pt_obj(r,object,numpoints1,4,PSP_SRCH_MTD,70.0, PSP_BTSET) && i<numpoints1)
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

int psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r,  int numsegs, int numlayers,  int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point,objCenter;
  double refObjAngle;
  double x,y,xo,yo,zo;
  configPt qcurr, qaux, taskqcurr;
  int i,resTask;
  psp_lst_vertex *lstvertex;
  float qQual, *QQuals, maxQual;
  int  besti, response;

  //  double kcd_with_report=0;
  double taskValue,bestTaskVal;

  QQuals = (float*) malloc(sizeof(float)*(numsegs+2)*numlayers);
  //vIndex = malloc(sizeof(float)*numsegs*numlayers);
  PSP_DRAW_QS = FALSE;

  lstvertex=&lstvert;
  //p3d_get_object_center(object, objCenter);
  objCenter[0] = sball->position[0];
  objCenter[1] = sball->position[1];
  objCenter[2] = sball->position[2];
  objCenter[3] = 0;

  globaljnt = FALSE;
  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r);
  taskqcurr = p3d_get_robot_config(r);
  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;
  destIndex = -1;
  ChronoOn();
  //lstvertex =  MY_ALLOC(psp_lst_vertex_tmp,1);
  //lstvertex =  MY_ALLOC(psp_lst_vertex,1);
  //if (!lstvertex)
  //  printf("NON alloc 1\n");
  // lstvertex->vertex = MY_ALLOC(psp_obs_vertex,numpoints);
  //if (!lstvertex->vertex)
  //  printf("NON alloc 2\n");
  InitWaveCells(r->env->box.x1,r->env->box.y1,r->env->box.x2,r->env->box.y2, qaux[ROBOTq_X], qaux[ROBOTq_Y],PSP_BTSET);
  //printCombinedBitmap();
  //psp_init_lst_vertex_obj(&lstvertex, numpoints , 1);
  //printGridVals();
  //printCombinedBitmap();
  //printObstacles();
  psp_gen_point_list_searchball (sball, r, numsegs, numlayers,lstvertex, PSP_BTSET);
  ox = xo = objCenter[0];
  oy = yo = objCenter[1];
  oz = zo = objCenter[2];
  sphereActive = 1;
  theqs = (double**) realloc(theqs,sizeof(configPt*)*lstvert.nv);

  if (search_method[PSP_SRCHM_TYPE] == PSP_ORDERED)
  {
    printf("Search Type: ORDERED\n");
    psp_order_point_list(lstvertex);
    printf("List Ordered\n");
  }
  if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM)
    srand((unsigned)time(0));
  qindex=0;
  testedQs=0;
  for (i=0;i<lstvertex->nv;i++)
  {

    if(psp_get_next_ordered_point(point,lstvertex))
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

	    //printf("x -> %f , %f\n",x,xo);
	    //printf("y -> %f , %f\n",y,yo);
	    //printf("z -> %f \n",zo);

	    ///Horizontal robot angle
      refObjAngle = rad_angleOf(x,y,xo,yo);

      //printf("Angle  %f", refObjAngle);
      qaux[ROBOTq_RZ] = angleLim(refObjAngle);

      printf("coord qaux -> %f , %f , %f\n",qaux[ROBOTq_X],qaux[ROBOTq_Y],qaux[ROBOTq_RZ]);

      qQual = psp_test_qs(r, qaux, qcurr,objCenter,viewPercent,1, PSP_BTSET);

      printf("--coord qaux -> %f , %f , %f\n",qaux[ROBOTq_X],qaux[ROBOTq_Y],qaux[ROBOTq_RZ]);

      testedQs++;

      if(qQual >-1)
      {
        if (PSP_NEXT_TASK != PSP_NO_TASK)
		    {
		      //p3d_set_and_update_this_robot_conf(r,qaux);
		      p3d_copy_config_into(r,qaux,&taskqcurr);
		      printf("coord tasqcurr -> %f , %f , %f\n",taskqcurr[ROBOTq_X],taskqcurr[ROBOTq_Y],taskqcurr[ROBOTq_RZ]);
		      //sphereActive =1;
		      //printf("Searching for task...\n");
		      if ((resTask=psp_take_it_at(r, sball->position, &taskqcurr,&taskValue)))
          {
            printf("YES %f\n",taskValue);

          }
		      else
          {
            printf("NO %f\n",taskValue);
          }

		      // taskValue = 0;	//to comment if you uncomment above
		      // resTask = 1;	//to comment if you uncomment above
		    }
        else
		    {
		      printf("NO NEXT TASK\n");
		      taskValue = 0;
		      resTask = 1;
		    }

        if (taskValue>-1 && resTask)
		    {
		      //if(!p3d_col_test_robot(r,kcd_with_report))
		      if (search_method[PSP_SRCHM_GOAL] ==  PSP_FFFO)
          {
            p3d_set_and_update_this_robot_conf(r,qcurr);
            theqs[qindex]  =  p3d_copy_config(r,qaux);
            qindex++;
            g3d_refresh_allwin_active();
            p3d_destroy_config(r,qaux);
            p3d_destroy_config(r,qcurr);
            destIndex = i;
            free(QQuals);
            ChronoPrint("PSP - TIME");
            ChronoOff();
            sphereActive = 0;
            return TRUE;
          }
		      else
          {
            printf("Storing Conf. %i\n",i);
            //theqs[qindex]  =  p3d_copy_config(r,qaux);

            if (PSP_NEXT_TASK != PSP_NO_TASK)
              theqs[qindex]  =  p3d_copy_config(r,taskqcurr);
            else
              theqs[qindex]  =  p3d_copy_config(r,qaux);

            //printf("Stored\n");
            QQuals[qindex] =  qQual;
            printf("Stored Quality list\n");
            if (qindex == 0)
            {
              besti = qindex;
              maxQual = qQual;
              bestTaskVal = taskValue;
            }
            else
            {

              if (PSP_NEXT_TASK != PSP_NO_TASK)
              {
                if (abs(qQual-maxQual)<5 && taskValue<bestTaskVal)
                {
                  besti   = qindex;
                  maxQual = qQual;
                  bestTaskVal = taskValue;
                  destIndex = i;
                }
              }
              else
              {

                if (qQual>maxQual)
                {
                  besti   = qindex;
                  maxQual = qQual;
                  destIndex = i;
                  //bestTaskVal = taskValue;
                }
              }
            }
            qindex++;
          }
		    }
        else
          printf("Not feasible task in %i \n",i);
      }


    }
    else
      printf("Point %i out of range\n",i);
  }//end for

  p3d_set_and_update_this_robot_conf(r,qcurr);

  if (qindex==0)
  {
    printf("---- ERROR: Not modeling point found -----\n");
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
    p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
    response = FALSE;
  }

  else
  {
    printf("---- %i Configurations  Generated -----\n",lstvert.nv);
    printf("---- %i Configurations  Tested    -----\n",testedQs);
    printf("---- %i Configurations  Found     -----\n",qindex);
    printf("---- Best configuration  on %i with %f in %f %f  -----\n", besti, maxQual,theqs[besti][ROBOTq_X],theqs[besti][ROBOTq_Y]);
    p3d_set_and_update_this_robot_conf(r,theqs[besti]);
    p3d_copy_config_into(r,theqs[besti], &(r->ROBOT_GOTO));
    //destIndex = besti;
    //p3d_set_and_update_this_robot_conf(r,qcurr);
    response = TRUE;
  }
  printf("freeing\n");
  p3d_destroy_config(r,qaux);
  printf("freein2\n");
  p3d_destroy_config(r,qcurr);
  printf("ALL free\n");
  g3d_refresh_allwin_active();
  ChronoPrint("PSP - TIME");
  ChronoOff();
  sphereActive = 0;
  return response;

}

int psp_srch_model_pt_at_point(double x, double y, double z, double dmin, double dmax, p3d_rob* r,  int numsegs, int numlayers,  int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET)
{
  p3d_psp_set_search_ball_pos(x,y,z);
  srchball.distMin = dmin;
  srchball.distMax = dmax;

  return psp_srch_model_pt_searchball(&srchball,r,numsegs,numlayers,search_method,viewPercent,PSP_BTSET);

}


/*
 int psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r,  int numsegs, int numlayers, psp_lst_vertex *lstVtx, hri_bitmapset* PSP_BTSET)
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
 */
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



int psp_srch_rnd_model_pt(p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET)
{
  p3d_vector4 point,v_aux;
  //p3d_jnt *jntPt = objRob->joints[1];
  //p3d_jnt *jntPt2;
  //p3d_jnt *jntPtCam = r->joints[r->cam_body_index];
  //int i_DoF;
  double f_max,refHumAngle;// lastAngle, lastAngleCam; f_min,
  double dMax,dMin,tMax,tMin,x,y,xo,yo,zo;
  //double supVLim, infVLim, centerV;
  configPt qcurr, qaux,objqcurr;
  int i;
  //int res;

  srand((unsigned)time(0));

  dMax =  objRob->max_pos_range;
  dMin =  objRob->min_pos_range;
  tMax =  0.0;
  tMin =  M_PI;


  qcurr    =  p3d_get_robot_config(r);
  qaux     =  p3d_get_robot_config(r);
  objqcurr =  p3d_get_robot_config(objRob);
  point[0] = 1.0;
  point[1] = 1.0;
  point[3] = 1.0;

  v_aux[0] = 0.0;
  v_aux[1] = 0.0;
  v_aux[2] = 0.0;


  for (i=0;i<numpoints;i++)
  {

    //Random point type
    psp_gen_rand_point(point,dMax,dMin,tMax,tMin);
    psp_get_RTH_to_XY_rframe(objRob,point, v_aux);
    printf("Point generated %i - %f,%f\n",i,point[0],point[1]);

    // passing from local to global coords
    //p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);


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

	  //qaux[ROBOTq_RZ] = angleLim(refHumAngle);


	  if(psp_test_qs(r, qaux, qcurr,point,80.0,1, PSP_BTSET))
	    return TRUE;
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



static void psu_get_point_ahead_cam(p3d_rob* rob, double radius, p3d_vector4 point)
{
  int axe = rob->cam_axe;
  double x,y,z;
  double x2,y2,z2;
  double pan, tilt;
  p3d_vector4 point2;

  x    = rob->cam_pos[0];
  y    = rob->cam_pos[1];
  z    = rob->cam_pos[2];
  pan  = rob->cam_pan;
  tilt = rob->cam_tilt;

  switch (axe)
  {
    case 0:// X
      z2 = z + radius * cos(pan) * cos(tilt);
      y2 = y + radius * sin(tilt);
      x2 = x + radius * sin(pan);

      break;
    case 1:// Y
      y2 = y + radius * cos(pan) * cos(tilt);
      x2 = x + radius * sin(tilt);
      z2 = z + radius * sin(pan);
      break;
    case 2:// Z
    default:
      x2 = x + radius * cos(pan) * cos(tilt);
      y2 = y + radius * sin(tilt);
      z2 = z + radius * sin(pan);

  }

  point2[0] = x2;
  point2[1] = y2;
  point2[2] = z2;
  point2[3] = 1.0;

  p3d_matvec4Mult(rob->o[rob->cam_body_index]->jnt->abs_pos,point2,point);


}

/**********************************************************************/

// Perspective reasonning  functions


/**********************************************************************/

int psu_get_num_objects_near(p3d_rob *currRob, double radius, int type, p3d_obj **oList)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 objCenter,robCenter;
  int i,j;
  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int contObj =0;
  p3d_vector4 pointHead,pointDisc, pointAhead;
  psp_cone cone;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_get_robot_center(currRob, robCenter);

  //mocap_get_disc_point(pointDisc);
  psu_get_point_ahead_cam(currRob, radius, pointAhead);


  p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);

  lx1 = pointHead [0];
  ly1 = pointHead [1];
  lz1 = pointHead [2];
  //printf("point Head %f %f %f \n", pointHead [0],pointHead [1],pointHead [2] );
  lx2 = pointAhead [0];
  ly2 = pointAhead [1];
  lz2 = pointAhead [2];
  //printf("point ahead %f %f %f \n", pointAhead [0],pointAhead [1],pointAhead [2] );

  psp_deselect_all();
  //  sphereActive = 1;
  //Static Obstacles
  for(i=0;i<no;i++)
  {
    o = envPt->o[i];
    if (!strcasestr(o->name,"FURN"))
    {
      p3d_get_object_center(o,objCenter);

      if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	    {
	      if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, 0.7, &disttocenter))
        {
          PSP_DRAW_OBJ_ARRAY [contObj] = i;
          //o->caption_selected = 1;
          oList[contObj] = o;
          contObj++;
        }
	    }
    }
  }

  //Robot body parts

  for(i=0;i<nr;i++)
  {
    r = envPt->robot[i];
    for(j=0;j<r->no;j++)
    {
      o = r->o[j];
      p3d_get_object_center(o,objCenter);
      if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	    {
	      if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, 1.0, &disttocenter))
        {
          //PSP_DRAW_OBJ_ARRAY [contObj] = j;
          //o->caption_selected = 1;
          //printf ("Selecting the robot part %s \n", o->name);
          oList[contObj] = o;
          contObj++;
        }
	    }


    }

    //define if robot is near or not? here or in observation? od we need a different list
    // if ((ContObjTmp/r->no)>.4)
    //

  }

  return contObj;
}


/**********************************************************************/

// Perspective reasonning  functions


/**********************************************************************/

int psu_get_num_objects_near_limited(p3d_rob *currRob, double radius, int type, double limDist, p3d_obj **oList, double *distances)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 objCenter,robCenter;
  int i,j;
  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int contObj =0;
  p3d_vector4 pointHead, pointAhead;
  //psp_cone cone;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_get_robot_center(currRob, robCenter);

  //mocap_get_disc_point(pointDisc);
  psu_get_point_ahead_cam(currRob, radius, pointAhead);


  p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);

  lx1 = pointHead [0];
  ly1 = pointHead [1];
  lz1 = pointHead [2];
  //printf("point Head %f %f %f \n", pointHead [0],pointHead [1],pointHead [2] );
  lx2 = pointAhead [0];
  ly2 = pointAhead [1];
  lz2 = pointAhead [2];
  //printf("point ahead %f %f %f \n", pointAhead [0],pointAhead [1],pointAhead [2] );

  psp_deselect_all();
  //  sphereActive = 1;
  //Static Obstacles
  if (type==0 || type==2)
    for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      if (!strcasestr(o->name,"furn"))
      {
        p3d_get_object_center(o,objCenter);

        if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	      {
          if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, 0.7, &disttocenter))
          {
            PSP_DRAW_OBJ_ARRAY [contObj] = i;
            //o->caption_selected = 1;
            oList[contObj] = o;
            distances [contObj] = disttocenter;
            contObj++;
          }
	      }
      }
    }

  //Robot body parts
  if (type==1 || type ==2)
    for(i=0;i<nr;i++)
    {
      r = envPt->robot[i];
      for(j=0;j<r->no;j++)
      {
        o = r->o[j];
        //if (strcasestr(o->name,"head") || strcasestr(o->name,"hand") )
        // {
        p3d_get_object_center(o,objCenter);
        if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	      {
          if (p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, 1.0, &disttocenter))
          {
            //PSP_DRAW_OBJ_ARRAY [contObj] = j;
            //o->caption_selected = 1;
            //printf ("Selecting the robot part %s \n", o->name);
            if(disttocenter <= limDist)
            {
              oList[contObj] = o;
              contObj++;
            }
          }
	      }
        // }
      }

      //define if robot is near or not? here or in observation? od we need a different list
      // if ((ContObjTmp/r->no)>.4)
      //

    }

  return contObj;
}






/**********************************************************************/

//


/**********************************************************************/

int psu_get_num_robots_near(p3d_rob *currRob, double radius, int type, p3d_rob **rList)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 rCenter,robCenter;
  int i,j,nr = envPt->nr;
  p3d_rob *r;
  p3d_obj *o;
  int contObj =0;
  p3d_vector4 pointHead,pointDisc, pointAhead;
  psp_cone cone;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_get_robot_center(currRob, robCenter);


  p3d_deselect_all_objects();
  //  sphereActive = 1;

  for(i=0;i<nr;i++)
  {
    r = envPt->robot[i];
    p3d_get_robot_center(r,rCenter);

    if(linearDistance(robCenter[0],robCenter[1], rCenter[0], rCenter[1])<=radius)
    {
      //mocap_get_disc_point(pointDisc);
      p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);
      psu_get_point_ahead_cam(currRob, radius, pointAhead);

      lx1 = pointHead [0];
      ly1 = pointHead [1];
      lz1 = pointHead [2];

      lx2 = pointAhead [0];
      ly2 = pointAhead [1];
      lz2 = pointAhead [2];

      // printf("point ahead %f %f %f \n", ox,oy,oz );
      if (p3d_psp_is_point_in_a_cone(rCenter, pointHead, pointAhead, 0.7, &disttocenter))
	    {
	      for(j=0;j<r->no;j++)
        {
          o = r->o[i];
          p3d_get_object_center(o,rCenter);
          if (p3d_psp_is_point_in_a_cone(rCenter, pointHead, pointAhead, 0.7, &disttocenter))
          {
            PSP_DRAW_OBJ_ARRAY [contObj] = i;
            //o->caption_selected = 1;
            rList[contObj] = r;
            contObj++;
          }
        }


	    }

    }
  }
  return contObj;
}

/****************************************************************/
/*!
 * \brief Generates a list of observed objects
 *
 * !
 */
/****************************************************************/

void psr_get_obj_list(p3d_rob *currRob, p3d_obj **oList, int *nObj,  p3d_rob **rList, int *nRob, double viewPercent)
// listas como parametros para despues comparar joint attention
// checar el robot actual para verificar que no intente verse solo
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_obj *o;
  p3d_rob *r, *human;
  double perspValue;
  int no = envPt->no;
  p3d_vector4 objCenter,robCenter;
  int i, obsR=0, obsO=0;
  double radius = 3.0;

  // rList = MY_ALLOC(p3d_rob*,envPt->nr);
  //oList = MY_ALLOC(p3d_obj*,envPt->no);

  p3d_get_robot_center(currRob, robCenter);

  printf("------------ Robots: %i\n", envPt->nr);

  p3d_deselect_all_objects();

  for(i=0;i<envPt->nr;i++)
  {
    p3d_select_robot_to_view(envPt->robot[i]);
    if(pso_look_obj())
      //      if(pso_watch3_obj()>
    {
      printf("Robot observed: %s\n",  envPt->robot[i]->name);
      rList[obsR] = envPt->robot[i];
      obsR++;
    }
    p3d_deselect_robot_to_view(envPt->robot[i]);

    /*
     if (strcasestr(envPt->robot[i]->name,"HUMAN"))
     {
     human = envPt->robot[i];
     }
     else
     if(strcasestr(envPt->robot[i]->name,"ROBOT"))
     {
     r = envPt->robot[i];
     }
     */
    // envPt->robot[i]->caption_selected = FALSE;
  }


  printf("------------ Objects: %i\n", no);

  p3d_deselect_all_objects();

  //search for obstacles in the environment
  for(i=0;i<no;i++)
  {
    o = envPt->o[i];
    if (!strcasestr(o->name,"FURN"))
    {
      //p3d_select_object_to_view(&o);
      p3d_get_object_center(o,objCenter);

      if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	    {
	      o->caption_selected = 1;
	      //printf("testing\n");
	      perspValue = pso_watch3_obj();
	      if (isnan(perspValue))
          perspValue = 0.0;
	      //printf("tested 1\n");
	      if(perspValue>=viewPercent)
        {
          printf("Object %s observed: %f\% \n", o->name,perspValue);
          oList[obsO] = o;
          obsO++;
        }
	      //p3d_unselect_object_to_view(&o);
	      o->caption_selected = 0;
	    }
    }
    //printf("tested 2/n");
  }

  *nObj =  obsO;
  *nRob =  obsR;

}
/****************************************************************/
/*!
 * \brief Generates a list of observed objects
 *
 * !
 */
/****************************************************************/

void psr_get_obj_list_multi(p3d_rob *currRob, p3d_obj **oList, int nObj, p3d_obj **oListOut, int *nObjOut, double viewPercent)
{
  p3d_obj *o;
  double *perspValues  = MY_ALLOC(double,nObj); // MY_ALLOC(double,PSP_NUM_OBJECTS) ;

  int i, obsO=0;

  PSP_CURR_DRAW_OBJ=0;
  //search for obstacles in the environment
  if( nObj > 0)
  {
    if (pso_watch_multi_obj(nObj, perspValues,oList))
    {
      //printf("At least 1\n");
      for(i=0;i<nObj;i++)
	    {
	      o = oList[i];//envPt->o[PSP_DRAW_OBJ_ARRAY [i]];
                     //printf("Object %s observed: %f\% \n", o->name,perspValues[i]);
	      if(perspValues[i]>=viewPercent)
        {
          //printf("---Object %s observed: %f\% \n", o->name,perspValues[i]);
          oListOut[obsO] = o;

          obsO++;
        }
	      else
          o->caption_selected = 0;
	    }

    }
    else
      psp_deselect_all();
  }
  else
    psp_deselect_all();



  //printf("----Objects in observed  %i\n",obsO);
  *nObjOut =  obsO;
  MY_FREE(perspValues, double, nObj);


}

int psr_set_attention_object(p3d_rob *rob, p3d_rob *human,  p3d_obj **oList, int nObj)
{

  int i;
  double xAvg=0, yAvg=0, zAvg=0;
  p3d_vector4 objCenter;
  configPt resq;
  configPt qcurr;
  resq  = p3d_get_robot_config(rob);
  qcurr = p3d_get_robot_config(rob);
  if (nObj > 0)
  {
    for (i=0;i<nObj;i++)
    {
      p3d_get_object_center(oList[i],objCenter);
      xAvg += objCenter[0];
      yAvg += objCenter[1];
      zAvg += objCenter[2];
    }

    xAvg /= nObj;
    yAvg /= nObj;
    zAvg /= nObj;
    //printf("looking at %f %f %f \n", xAvg, yAvg, zAvg);
  }
  else
  {
    p3d_get_object_center(human->o[human->cam_body_index], objCenter);
    xAvg = objCenter[0];
    yAvg = objCenter[1];
    zAvg = objCenter[2];
    //printf("looking at head in  %f %f %f \n", xAvg, yAvg, zAvg);
  }

  //if(psp_look_at(rob, xAvg, yAvg, zAvg, &resq))
  objCenter[0] = xAvg;
  objCenter[1] = yAvg;
  objCenter[2] = zAvg;
  objCenter[3] = 1;
  if(psp_set_pan_tilt(rob, objCenter, resq))
  {
    p3d_set_and_update_this_robot_conf(rob,resq);
    i = 1;
  }
  else
  {
    p3d_set_and_update_this_robot_conf(rob,qcurr);
    i =0;
  }

  free(resq);
  free(qcurr);

  return i;


}


void psr_get_rob_parts_list(p3d_rob *currRob, p3d_obj **partsList, int *nParts, double viewPercent)
{


}

void psr_get_joint_attention(hri_bitmapset* PSP_BTSET, double viewPercent)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *robot = (PSP_BTSET->robot);
  p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;
  //actual
  p3d_obj **oList;
  p3d_rob **rList;
  int nRob,nObj;
  //joint with
  p3d_obj **oListJnt;
  p3d_rob **rListJnt;
  int nRobJnt,nObjJnt;
  int i,j;

  ChronoOn();


  //psu_set_num_obj_255();

  nObj = envPt->no;
  nObjJnt = envPt->no;

  rList = MY_ALLOC(p3d_rob*,envPt->nr);
  oList = MY_ALLOC(p3d_obj*,envPt->no);
  rListJnt = MY_ALLOC(p3d_rob*,envPt->nr);
  oListJnt = MY_ALLOC(p3d_obj*,envPt->no);

  PSP_NUM_OBJECTS =  psu_get_num_objects_near(robot, 3.0, 0,oList);

  printf("Objects Robot = %i\n",PSP_NUM_OBJECTS);
  PSP_ROBOT = robot;
  psr_get_obj_list_multi(PSP_ROBOT, oList,nObj,oList,&nRob,viewPercent); //AKINFIX rlist --> olist


  PSP_NUM_OBJECTS =  psu_get_num_objects_near(human, 3.0, 0,oListJnt);

  printf("Objects Human = %i\n",PSP_NUM_OBJECTS);

  PSP_ROBOT = human;
  psr_get_obj_list_multi(PSP_ROBOT, oListJnt,nObjJnt,oListJnt,&nRobJnt,viewPercent); //AKINFIX rlist --> olist

  /*
   for (i = 0; i<nObj; i++)
   {
   oList[i]->caption_selected = TRUE;
   }
   *

   for (i = 0; i<nObjJnt; i++)
   {
   oListJnt[i]->caption_selected = TRUE;
   }
   */
  p3d_deselect_all_objects();
  for (i = 0; i<nObj; i++)
  {
    for (j = 0; j<nObjJnt && !oList[i]->caption_selected; j++)
    {
      if (oList[i]->o_id == oListJnt[j]->o_id)
	    {
	      oList[i]->caption_selected = TRUE;
	    }
    }
  }

  ChronoPrint("PSP JOINT ATTENTION - TIME");
  ChronoOff();
  free(oList);
  free(oListJnt);
  PSP_ROBOT = robot;
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

    //if (psp_srch_model_pt_searchball(&srchball,r, 40,60, PSP_BTSET))
    //  {
    printf("search ball found\n");
    // }

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



      //ojo cambiaron los parametros de esta funcion
      //psp_srch_model_pt_obj(r,oSel, 60,4,80.0, PSP_BTSET);
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

static int psu_get_index_from_color(float colorIdx)
{

  int i;
  float differ;
  float factor = (PSP_MAX_COLOR_IDX/PSP_NUM_OBJECTS*1.0)/2.0;
  //printf("Objects: %i\n",PSP_NUM_OBJECTS);
  for (i=0;i<PSP_NUM_OBJECTS;i++)
  {
    differ = fabs(PSP_DRAW_OBJ_COL_INDEX[i]-colorIdx);
    //printf("Difference %i => %f - %f = %f\n",i, colorIdx, PSP_DRAW_OBJ_COL_INDEX[i], differ);
    if (differ <= factor)
    {
      //printf("Difference %i => %f - %f = %f\n",i, colorIdx, PSP_DRAW_OBJ_COL_INDEX[i], differ);
      return i;
    }
  }

  return -1;
}

static void psu_set_num_obj_255()
{


  while (255%PSP_NUM_OBJECTS != 0)
  {
    PSP_NUM_OBJECTS++;
  }


}


/***************************************
 watch_multi = gives object observation % it reduces computation time with the size reduction on the second buffer
 ****************************************/

static int pso_watch_multi_obj(int numObj,double *percentages, p3d_obj **oList)
{

  //int        w=0,h=0;
  int        w=133,h=66;
  G3D_Window *win = g3d_get_win_by_name((char*)"Perspective");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);

  int        i,j, *greenCount, *totalCount;
  int oneatleast = FALSE;
  int firsti=-1, lasti=-1;
  int curridx;
  //float factor = (PSP_MAX_COLOR_IDX/PSP_NUM_OBJECTS*1.0);

  //double factorb = 255/PSP_NUM_OBJECTS;
  //FILE * mapColor = fopen("mapcolor.dat","w");


  //fl_get_winsize(FL_ObjWin(ob),&w,&h);
  G3D_RESFRESH_PERSPECTIVE = FALSE;
  PSP_REFRESH_BLOCK = FALSE;

  glDrawBuffer (GL_BACK);//draw window function makes swap, coping back to front
  glReadBuffer(GL_BACK) ;

  GLfloat* pixels = MY_ALLOC(GLfloat,(w*h*3));

  //GLubyte* pixels = MY_ALLOC(GLubyte,(w*h*3));
  //percentages =  MY_ALLOC(double,numObj);
  totalCount =   MY_ALLOC(int,numObj);
  greenCount =   MY_ALLOC(int,numObj);


  for (i=0;i<numObj;i++)
  {
    percentages[i] = 0.0;
    totalCount[i]  = 0.0;
    greenCount[i]  = 0.0;
    //printf("obj %i color %f\n",i,PSP_DRAW_OBJ_COL_INDEX[i]);
  }


  for (i=0;i<(h*w*3);i+=3)
  {
    pixels[i]=0.0;
  }

  PSP_NUM_OBJECTS = 1;
  PSP_CURR_DRAW_OBJ=0;

  glLoadIdentity();
  g3d_set_win_draw_mode(win,OBJECTIF);
  psp_deselect_all();

  for (j=0; j<numObj;j++)
  {
    oList[j]->caption_selected = 1;
    //printf("Analizing obj %s\n",oList[j]->name);
    //glLoadIdentity();
    //g3d_refresh_win(win);
    canvas_expose_special(ob, NULL, w, h, NULL, win);
    glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT, pixels);


    for (i=0;i<(h*w*3);i=(i+3))
    {
      //if (strcasestr(oList[j]->name,"compu"))
      //  fprintf(mapColor,"%i %i %f",i%(w*3),pixels[i+1]);

      if (pixels[i+1]>0.0000002)
	    {
	      //Green pixels
	      if (firsti==-1)
          firsti=i;

	      lasti=i;

	      //curridx = (int)((round(pixels[i+1]*100)/100)/factor);

	      //curridx -=1;
	      //curridx = psu_get_index_from_color(pixels[i+1]);
	      //if (curridx >=0)
	      //	{
	      totalCount[j] ++;
	      //printf("------%i = %i\n", curridx,totalCount[curridx]);
	      //}

	      //printf("Pixel %i -> %f,%f,%f\n",i,pixels[i],pixels[i+1],pixels[i+2]);
	    }

    }
    //printf("analizing %s with index %i total=%i\n",oList[j]->name,j, totalCount[j]);
    oList[j]->caption_selected = 0;
  }


  PSP_NUM_OBJECTS = numObj;
  for (j=0; j<numObj;j++)
  {
    oList[j]->caption_selected = 1;
  }

  //printf("Total \n");
  //printf("Indexes %i -> %i\n",firsti, lasti);


  glLoadIdentity();
  g3d_set_win_draw_mode(win,DIFFERENCE);
  //g3d_refresh_win(win);
  canvas_expose_special(ob, NULL, w, h, NULL, win);
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,pixels);
  //decodificar cada pixel
  for (i=0;i<(h*w*3);i+=3)
    //for (i=firsti;i<=lasti;i++)
  {
    //      if (pixels[i]>=0.0 && pixels[i]!=1.0)
    //	{
    if (pixels[i+1]>0.0000002)
    {
      //Green pixels
      //curridx = (int)((round(pixels[i+1]*100)/100)/factor);
      curridx = psu_get_index_from_color(pixels[i+1]);
      if (curridx >=0)
	    {
	      greenCount[curridx] += 1;
	      //printf("-**--- %i = %i\n",curridx,greenCount[curridx] );
	    }
      //if (curridx == 5)
      //printf("Pixel %i -> %i,%i,%i\n",i,pixels[i],pixels[i+1],pixels[i+2]);
      // printf("green - index: %i count: %i\n",curridx, greenCount[curridx]);

    }

    //	}
  }

  //printf("green \n");

  for (i=0;i<numObj;i++)
  {
    if (totalCount[i]>0.0)///(h*w*1.0)>.005)//totalCount[i]>0)// && totalCount[i]/(h*w)>.15)
    {
      percentages[i] = (greenCount[i]*100.0)/(totalCount[i]*1.0);
      oneatleast = TRUE;
    }
    else
    {
      percentages[i] = 0.0;
      //printf("--\n");
    }

    // printf("percentages %i = %f (%i / %i) of %f \n",i,totalCount[i]/(h*w*1.0), greenCount[i],totalCount[i],(h*w) );
  }

  G3D_RESFRESH_PERSPECTIVE = TRUE;
  PSP_REFRESH_BLOCK = TRUE;

  //printf("freeing \n");
  MY_FREE(pixels,GLfloat,w*h*3);
  //MY_FREE(pixels,GLubyte,w*h*3);
  MY_FREE(greenCount,int,numObj);
  MY_FREE(totalCount,int,numObj);

  //printf("refreshing \n");
  g3d_set_win_draw_mode(win,NORMAL);
  //g3d_refresh_win(win);

  //printf("Going out \n");

  return oneatleast;
}




/***************************************
 watch3 = gives object observation % it reduces computation time with the size reduction on the second buffer
 ****************************************/

double pso_watch3_obj()
{

  int        w=133,h=66;
  //GLint viewport[4];
  G3D_Window *win = g3d_get_win_by_name((char*)"Perspective");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);
  //fl_get_winsize(FL_ObjWin(ob),&w,&h);


  G3D_RESFRESH_PERSPECTIVE = FALSE;

  int        i, greenCount=0, totalCount=0;
  double total=0.0;
  int firsti=-1, lasti=-1;

  glDrawBuffer (GL_BACK);//draw window function makes swap coping back to front
  glReadBuffer(GL_BACK) ;
  GLfloat* pixels = MY_ALLOC(GLfloat,(w*h*3));

  for (i=0;i<(h*w*3);i+=3)
  {
    pixels[i]=0.0;
  }

  glLoadIdentity();
  g3d_set_win_draw_mode(win,OBJECTIF);

  //g3d_refresh_win(win);
  canvas_expose_special(ob, NULL, w, h, NULL, win);

  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT, pixels);


  for (i=0;i<(h*w*3);i=(i+3))
  {
    if (pixels[i]>=0.0 && pixels[i]!=1.0)
    {
      if ((pixels[i+1]>0.0) &&  (pixels[i+2]==0.0))
	    {
	      //Green pixels
	      if (firsti==-1)
          firsti=i;

	      lasti=i;

	      totalCount++;
	      //printf("1");
	    }

      // printf("Blue\n");
      //printf("Pixel %i -> %f,%f,%f\n",i,pixels[i],pixels[i+1],pixels[i+2]);

    }
    //else
    //  printf(".");
    //if(i/w==1)
    //  printf("_ \n");


  }

  //printf("Indexes %i -> %i = %i\n",firsti, lasti,totalCount);
  glLoadIdentity();
  g3d_set_win_draw_mode(win,DIFFERENCE);
  //g3d_refresh_win(win);
  canvas_expose_special(ob, NULL, w, h, NULL, win);
  glReadPixels(0,0,w,h,GL_RGB,GL_FLOAT,pixels);
  //decodificar cada pixel
  //for (i=0;i<(h*w*3);i+=3)
  for (i=firsti;i<=lasti;i+=3)
  {
    if (pixels[i]>=0.0 && pixels[i]!=1.0)
    {
      if ((pixels[i+1]>0.0) &&  (pixels[i+2]==0.0))
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

  G3D_RESFRESH_PERSPECTIVE = TRUE;
  MY_FREE(pixels,GLfloat,w*h*3);
  g3d_set_win_draw_mode(win,NORMAL);
  //g3d_refresh_win(win);

  G3D_RESFRESH_PERSPECTIVE = TRUE;
  return total;
}

/***************************************
 watch = gives object observation %
 ****************************************/
static double pso_watch2_obj()
{

  int        w=0,h=0;
  G3D_Window *win = g3d_get_win_by_name((char*)"Perspective");
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
  G3D_Window *win = g3d_get_win_by_name((char*)"Perspective");
  FL_OBJECT  *ob = ((FL_OBJECT *)win->canvas);
  FL_FORM *tmpwin = NULL;
  fl_get_winsize(FL_ObjWin(ob),&w,&h);

  // float* pixels=malloc(sizeof(GL_FLOAT)*w*h*3);
  int        i, greenCount=0, totalCount=0;
  double total=0.0;


  tmpwin= ((FL_FORM *)win->form);
  fl_raise_form(tmpwin);


  if(glXGetCurrentContext() !=  (void*)fl_get_glcanvas_context(ob))
  {
    printf("Raising window -> %s\n",win->name);
    glXMakeCurrent(fl_display,FL_ObjWin(ob), fl_get_glcanvas_context(ob));
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
  if (pso_watch3_obj()>=15.0)
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

static void psp_gen_confs(int n, p3d_rob* r)
{
  int i;
  for (i=0; i<n; i++)
  {
    theqs[i] = p3d_get_robot_config(r);
  }
}

void initpspGiks() {
  PSP_GIK = hri_gik_create_gik();
  PSP_GIK2 = hri_gik_create_gik();
  PSP_GIK3 = hri_gik_create_gik();
}

int p3d_init_robot_parameters()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *currobotPt;
  int i;
  for(i=0; i<envPt->nr; i++){
    currobotPt=envPt->robot[i];
    if(strcasestr(currobotPt->name,"SUPERMAN")) //for all humans
    {
      ////////batman
      p3d_set_rob_cam_parameters(currobotPt,.1,.0,.0,3.0,7.0,1.0,2.0,15,2,.0,.0);
      ///////achiles
      //p3d_set_rob_cam_parameters(currobotPt,.0,-.10,.05,3.0,7.0,1.0,2.0,2,0,.0,-1.6);
      //give
      currobotPt->angle_range   = 2.0;
      //currobotPt->max_pos_range = 1.3; //3.0;
      currobotPt->min_pos_range = 1.2; //2.0;
                                       //talk
      currobotPt->max_pos_range = 4.0; //3.0;
    }
    else 
      if(strcasestr(currobotPt->name,"ACHILE")){
        p3d_set_rob_cam_parameters(currobotPt,.0,-.10,.05,3.0,7.0,1.0,2.0,5,0,0,-1.6);
        currobotPt->angle_range   = 2.0;
        //currobotPt->max_pos_range = 1.3; //3.0;
        currobotPt->min_pos_range = 1.2; //2.0;
                                         //talk
        currobotPt->max_pos_range = 4.0; //3.0;
      }
      else
        if(strcasestr(currobotPt->name,"ROBOT"))
        {
          
#ifdef HRI_JIDO
        p3d_set_rob_cam_parameters(currobotPt,.0,-.10,.0,3.0,7.0,0.75,1.05,12,2,.0,.0);
#elif defined HRI_TUM_BH
        p3d_set_rob_cam_parameters(currobotPt,.05,-.05,.0,3.0,7.0,0.75,1.05,4,2,.0,.0);
#elif defined HRI_HRP2
        //p3d_set_rob_cam_parameters(currobotPt,.05,-.10,.0,3.0,7.0,0.75,1.05,10,16,.0,.0);
        p3d_set_rob_cam_parameters(currobotPt,.15,-.10,.0,3.0,7.0,1.10,1.35,16,2,.0,.05);//hrp2
#elif defined HRI_BERT
        p3d_set_rob_cam_parameters(currobotPt,.15,-.10,.0,3.0,7.0,1.10,1.35,16,2,.0,.05);//hrp2
#endif
        currobotPt->angle_range   = 1.0;
        currobotPt->max_pos_range = 3.0;
        currobotPt->min_pos_range = 1.0;
        PSP_ROBOT = currobotPt;
      }
      else
      {
        currobotPt->angle_range   = .0;
        currobotPt->max_pos_range = 1.2;
        currobotPt->min_pos_range = 1;
      }
    //printf("Robot %i %s\n",i,envPt->robot[i]->name);

  }

  initpspGiks();
  //p3d_init_object_parameters_by_name("table1",0.5,1.0);
  p3d_init_object_parameters_by_name((char*)"HIGHTABLE_TABLE",0.8,1.3);
  p3d_init_object_parameters_by_name((char*)"TRASHBIN_TASH",0.8,1.3);
  p3d_init_object_parameters_by_name((char*)"LOWTABLE_TABLE",0.8,1.3);

  PSP_DEACTIVATE_AUTOHIDE =0;
  PSP_NUM_OBJECTS = 0;

  // printf ("--------------- this ---------------------\n");
  // psp_get_plan_utility("UPDATE: p1,p2,p3/x,y,z/you,and,me");
  //psp_get_plan_utility("PLAN: task1(1,2)/task2(1,2,3)/task3(1,2,3,4);");

  //PSP_MAX_COLOR_IDX = 1.0;
  //psp_gen_confs(40);
  return 0;
}

int p3d_init_object_parameters_by_name(char *objName,  double min, double max)
{

  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no, nr,i, cont=0;
  pp3d_obj o;
  pp3d_rob r;

  no = envPt->no;
  nr = envPt->nr;

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

  for(i=0;i<nr;i++)
  {
    r = envPt->robot[i];
    if (strcmp(r->name,objName)==0)
    {
      r->angle_range = 1;
      r->max_pos_range = max;
      r->min_pos_range = min;
      r->o[0]->max_pos_range = max;
      r->o[0]->min_pos_range = min;
      //printf(
      cont++;
    }
  }


  return cont;

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
    if (strcasestr(o->name,objType))
    {
      o->max_pos_range = max;
      o->min_pos_range = min;
      cont++;
    }
  }

  return cont;

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


int psp_srch_for_target_obj(p3d_rob *robot, int numsegs, int numlayers, int searchMode, int *searchMtd, double viewpercent, hri_bitmapset* PSP_BTSET )
{

  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int found;
  int no, i, nr;
  pp3d_obj o;
  pp3d_rob r;

  no = envPt->no;
  nr = envPt->nr;
  found = FALSE;
  //search for obstacles in the environment that aren't in seleted robot(s)
  for(i=0;i<no && !found;i++)
  {
    o = envPt->o[i];
    //found = FALSE;
    if(o->caption_selected)
    {
      found=TRUE;
    }
  }
  if (found)
  {
    //if (searchMode < 2)
    //{
    if (searchMode > 0)
      PSP_DEACTIVATE_AUTOHIDE = 1;

    if (psp_srch_model_pt_obj(robot,o,numsegs, numlayers, searchMtd, viewpercent, PSP_BTSET))
    {
      PSP_DEACTIVATE_AUTOHIDE = 0;
      return TRUE;
    }
    PSP_DEACTIVATE_AUTOHIDE = 0;
    //}
    //      else
    //if ( psp_goto_look_obj(robot,o,40,20,1,PSP_BTSET))
    // return TRUE;

  }
  else
  {
    for(i=0;i<nr && !found;i++)
    {
      r = envPt->robot[i];
      //found = FALSE;
      if (r!=robot)
        if(r->caption_selected)
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
	      printf("search for an object:\n");
	      if (psp_srch_model_pt_obj(robot,r->o[0],numsegs, numlayers, searchMtd, viewpercent, PSP_BTSET))
        {
          PSP_DEACTIVATE_AUTOHIDE = 0;
          return TRUE;
        }
	      PSP_DEACTIVATE_AUTOHIDE = 0;
	    }
      else
        if (psp_srch_model_pt(robot, r, numsegs, numlayers, searchMtd, viewpercent, PSP_BTSET))
          return TRUE;

    }

  }
  printf("**PSP** ERROR: NO selected object found\n");
  return FALSE;
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


void psp_deselect_all_robots()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr,i;
  pp3d_rob r;


  nr = envPt->nr;
  //robotPt->caption_selected = 0;

  for(i=0;i<nr;i++)
  {
    r = envPt->robot[i];
    p3d_deselect_robot_to_view(r);
  }

}

void psp_deselect_all()
{
  p3d_deselect_all_objects();

  psp_deselect_all_robots();
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
  pp3d_obj o=NULL;


  no = envPt->no;

  for(i=0;i<no && !cont;i++)
  {
    o = envPt->o[i];
    if (strcmp(o->name,objName)==0)
    {
      o->caption_selected = 1;
      cont=i;
    }
  }
  if (cont)
    printf("Object %s found %i time(s)\n",objName,cont);
  else
    printf("Object %s NOT found \n",objName);

  return cont;
}

pp3d_rob psp_select_robot_by_name(char *robName)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr,i, cont=0;
  pp3d_rob r=NULL;


  nr = envPt->nr;

  for(i=0;i<nr && !cont;i++)
  {
    r = envPt->robot[i];
    if (strcmp(r->name,robName)==0)
    {
      cont=i;
    }
  }
  if (cont)
    return r;
  else
    return NULL;
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
    g3d_drawColorSphere(srchbPt->pos[0],srchbPt->pos[1],srchbPt->pos[2], 0.2, Green,NULL);
    glPopMatrix();
  }
}





void p3d_psp_set_search_ball_pos( double x, double y, double z)
{
  srchball.position[0] = x;
  srchball.position[1] = y;
  srchball.position[2] = z;
}

/*
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
  pointc[3] = 0.0;
}

//Center point based on BB of the object
void p3d_get_object_center(p3d_obj* obj, p3d_vector4 pointc)
{
  pointc[0] = (((obj->BB.xmax - obj->BB.xmin)/2) + obj->BB.xmin);
  pointc[1] = (((obj->BB.ymax - obj->BB.ymin)/2) + obj->BB.ymin);
  pointc[2] = (((obj->BB.zmax - obj->BB.zmin)/2) + obj->BB.zmin);
  pointc[3] = 0.0;
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
 * \brief Limit angle to range [PI, -PI]
 *
 * \param angle - angle to validate
 *
 * !

 */
/****************************************************************/

double angleLim(double angle)
{
  if ( angle > M_PI)
    return (angle - (2*M_PI));
  return angle;

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

  double a     =  p3d_psp_pointtolinedist(paux,conepaux,conepaux2);//distance to cone axis
  double c     =  DISTANCE3D(conepaux[0],conepaux[1],conepaux[2],paux[0],paux[1],paux[2]); //distance to base
  double alfa  =  asin((a*sin(M_PI/1.57))/c);

  //printf("a: %f c:%f  alfa:%f \n", a,c,alfa);
  *distf = a;
  if (alfa<coneangle/2.0 && (disttoorigin<=distofline && c<=distofline))
    return 1;
  else
    return 0;

}






static void psp_draw_confs( G3D_Window  *win )
{
  int i;
  //G3D_Window* win = g3d_get_win_by_name((char*)"Move3D");
  //printf("cuantos %i\n",qindex);
  p3d_sel_desc_num(P3D_ROBOT, PSP_ROBOT->num);
  glPushMatrix();
  if (PSP_ROBOT)
    for (i=0; i<qindex; i++)
    {
      //glXWaitGL();
      p3d_set_and_update_this_robot_conf(PSP_ROBOT,theqs[i]);
      g3d_draw_robot(PSP_ROBOT->num,win);

      //printf("passing %i %f,%f\n",i,theqs[i][ROBOTq_X],theqs[i][ROBOTq_Y]);
    }
  p3d_set_and_update_this_robot_conf(PSP_ROBOT,PSP_ROBOT->ROBOT_POS);
  glPopMatrix();
}


void  psp_draw_elements( G3D_Window  *win)
{

  if (win->win_perspective) // These features are not shown in a perspective window
    return;

  double *color_vect = NULL;
  double radius;
  int i,j;
  p3d_vector4 auxpoint, rvertex;
  p3d_matrix4 matrix;
  //  double c    =  DISTANCE3D(lx1, ly1, lz1, lx2, ly2, lz2);

  // double lradz = sqrt((c*c)+(lrad*lrad));

  g3d_drawOneLine(lx1, ly1, lz1, lx2, ly2, lz2, Green, color_vect);

  /*   g3d_drawOneLine(lx1, ly1, lz1, lx2+lrad, lz2 , ly2+lrad,  Red, color_vect); */

  /*   g3d_drawOneLine(lx1, ly1, lz1, lx2-lrad, lz2, ly2+lrad,  Red, color_vect); */

  /*   g3d_drawOneLine(lx1, ly1, lz1, lx2-lrad, lz2, ly2-lrad,  Red, color_vect); */

  /*   g3d_drawOneLine(lx1, ly1, lz1, lx2+lrad, lz2, ly2-lrad,  Red, color_vect); */

  // g3d_drawColorSphere(lx2, ly2, lz2,lrad,Red, color_vect);

  if (sphereActive)
  {
    g3d_drawColorSphere(ox, oy, oz,0.1,Blue, color_vect);
    g3d_draw_srchball_pos_area(&srchball);
  }
  if (globaljnt)
  {
    for(i=0 ; i<=3 ; i++){
      for(j=0 ; j<=3 ; j++){
        matrix[i][j]=globaljnt->abs_pos[i][j];
      }
    }
    auxpoint[3] = 1;

    for (i=0; i<lstvert.nv; i++)
    {
      auxpoint[0] = lstvert.vertex[i].pos[0];
      auxpoint[1] = lstvert.vertex[i].pos[1];
      auxpoint[2] = lstvert.vertex[i].pos[2];

      p3d_matvec4Mult(matrix,auxpoint,rvertex);
      if (i==destIndex)
        radius =0.07;
      else
        radius =0.01;
      //g3d_drawColorSphere(lstvert.vertex[i].pos[0]+ox,lstvert.vertex[i].pos[1]+oy,lstvert.vertex[i].pos[2]+oz,0.01, tRed, NULL);
      if ( lstvert.vertex[i].status == PSP_St_NOT_IN_RANGE )
        g3d_drawColorSphere(rvertex[0],rvertex[1],rvertex[2], radius, tRed, NULL);
      else
        g3d_drawColorSphere(rvertex[0],rvertex[1],rvertex[2], radius, Blue, NULL);

    }
  }
  else
  {
    for (i=0; i<lstvert.nv; i++)
    {
      if ( lstvert.vertex[i].status == PSP_St_NOT_IN_RANGE )
	    {
	      //printf("red\n");
	      g3d_drawColorSphere(lstvert.vertex[i].pos[0]+ox,lstvert.vertex[i].pos[1]+oy,lstvert.vertex[i].pos[2]+oz,0.01, Red, NULL);
	    }
      else
	    {
	      //printf("blue\n");
	      g3d_drawColorSphere(lstvert.vertex[i].pos[0]+ox,lstvert.vertex[i].pos[1]+oy,lstvert.vertex[i].pos[2]+oz,0.1, Blue, NULL);

	    }
    }
  }
  if (PSP_DRAW_QS && qindex>0)
    psp_draw_confs(win);
}

void psp_draw_search_ball(psp_searchball *srchballpt)
{

  if (sphereActive)
  {
    glPushMatrix();
    g3d_drawColorSphere(srchballpt->position[0],srchballpt->position[1],srchballpt->position[2],0.1,Green, NULL);
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

static void psp_init_lst_vertex(psp_lst_vertex *lstVtx,  int numSegs, int numLays)
{
  //  int i;
  /*  lstVtx->vertex = realloc(lstVtx->vertex,sizeof(psp_obs_vertex)*numSegs*numLays);
   lstVtx->grid   = MY_ALLOC(psp_obs_vertex*,numSegs);
   for (i=0; i<numSegs; i++)
   {
   lstVtx->grid [i] =  MY_ALLOC(psp_obs_vertex, numLays);
   if (lstVtx->grid [i] == NULL)
   printf("no allocation on segment %i \n",i);
   //    lstVtx->grid [i] =  (lstVtx->grid[i],sizeof(psp_obs_vertex)*numLays);
   }*/
}

static void psp_init_lst_vertex_obj(psp_lst_vertex *lstVtx,  int numSegs, int numLays)
{
  //  int i;
  /*  lstVtx->vertex = MY_ALLOC(psp_obs_vertex,numSegs);
   lstVtx->grid   = NULL;
   MY_ALLOC(psp_obs_vertex*,numSegs);
   for (i=0; i<numSegs; i++)
   {
   lstVtx->grid [i] =  MY_ALLOC(psp_obs_vertex, numLays);
   if (lstVtx->grid [i] == NULL)
   printf("no allocation on segment %i \n",i);
   //    lstVtx->grid [i] =  (lstVtx->grid[i],sizeof(psp_obs_vertex)*numLays);
   }
   */
}
static void psp_free_lst_vertex(psp_lst_vertex *lstVtx)
{
  free(lstVtx->vertex);
  //free(lstVtx->grid);//not a good way of freeing this.
}

void printListVtx(psp_lst_vertex *lstVtx)
{
  int i,j;
  FILE * lstf;
  FILE * lstfcost;
  FILE * lstfutility;
  lstf     = fopen("psppointsObs.dat","w");
  lstfcost = fopen("psppointsCost.dat","w");
  lstfutility = fopen("psppointsUtil.dat","w");
  if ((lstf == NULL) || (lstfcost == NULL) || (lstfutility == NULL))
  {
    printf("Can not open file.\n");
    return;
  }


  for (i=0;i<=lstVtx->ns;i++)
  {
    for (j=0;j<lstVtx->nl;j++)
    {

      if(lstVtx->grid[i][j].status  == PSP_NOT_AVAILABLE)
	    {
	      fprintf(lstf,"%f %f -3.0 \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1]);
	      fprintf(lstfcost,"%f %f -3.0 \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1]);
	      fprintf(lstfutility,"%f %f -3.0 \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1]);
	    }
      else
	    {
	      fprintf(lstf,"%f %f %f \n", lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1],lstVtx->grid[i][j].quality);
	      fprintf(lstfcost,"%f %f %f \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1],lstVtx->grid[i][j].cost);
	      fprintf(lstfutility,"%f %f %f \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1],lstVtx->grid[i][j].utility);
	      //printf("%f %f %f \n",lstVtx->grid[i][j].pos[0],lstVtx->grid[i][j].pos[1],lstVtx->grid[i][j].cost);
	    }
      /*	  if(lstVtx->grid[i+1][j].status  == PSP_NOT_AVAILABLE)
       fprintf(lstf,"%f %f 0.0 \n",lstVtx->grid[i+1][j].pos[0],lstVtx->grid[i+1][j].pos[1]);
       else
       fprintf(lstf,"%f %f %f \n", lstVtx->grid[i+1][j].pos[0],lstVtx->grid[i+1][j].pos[1],lstVtx->grid[i+1][j].obsPercent);
       if(lstVtx->grid[i][j+1].status  == PSP_NOT_AVAILABLE)
       fprintf(lstf,"%f %f 0.0 \n",lstVtx->grid[i][j+1].pos[0],lstVtx->grid[i][j+1].pos[1]);
       else
       fprintf(lstf,"%f %f %f \n", lstVtx->grid[i][j+1].pos[0],lstVtx->grid[i][j+1].pos[1],lstVtx->grid[i][j+1].obsPercent);
       if(lstVtx->grid[i+1][j+1].status  == PSP_NOT_AVAILABLE)
       fprintf(lstf,"%f %f 0.0 \n",lstVtx->grid[i+1][j+1].pos[0],lstVtx->grid[i][j+1].pos[1]);
       else
       fprintf(lstf,"%f %f %f \n", lstVtx->grid[i+1][j+1].pos[0],lstVtx->grid[i][j+1].pos[1],lstVtx->grid[i][j+1].obsPercent);
       fprintf(lstf,"\n");*/
    }
    fprintf(lstf,"\n");
    fprintf(lstfcost,"\n");
    fprintf(lstfutility,"\n");
  }

  fclose(lstf);
  fclose(lstfcost);
  fclose(lstfutility);
}

void printQcosts(int *indexes, float *qcst, int numqs, psp_lst_vertex *lstVtx) //wey coordenadas de los puntos
{
  int i;
  FILE * lstf;
  lstf = fopen("pspqcosts.dat","w");
  if (lstf == NULL)
  {
    printf("Can not open file.\n");
    return;
  }

  for (i=0;i<numqs;i++)
  {
    fprintf(lstf,"%f %f %f \n",lstVtx->vertex[indexes[i]].pos[0],lstVtx->vertex[indexes[i]].pos[1],qcst[i]);
  }

  fclose(lstf);

}

void psp_chng_show_st()
{
  PSP_DRAW_QS = !PSP_DRAW_QS;
}

void psp_chng_show_sphere_st()
{
  sphereActive = !sphereActive;
}

void psp_chng_show_autohide_st()
{
  PSP_DEACTIVATE_AUTOHIDE = !PSP_DEACTIVATE_AUTOHIDE;
}


int psp_set_device_pos_by_name(char *devName, double x, double y, double z, double th)
{

  pp3d_rob r =  psp_select_robot_by_name(devName);

  if (r)
  {
    configPt qset =  p3d_get_robot_config(r);
    qset[MOBJECTq_X] = x;
    qset[MOBJECTq_Y] = y;
    qset[MOBJECTq_Z] = z;
    qset[MOBJECTq_RZ] = th;
    p3d_set_and_update_this_robot_conf(r, qset);
    p3d_destroy_config(r, qset);
    return TRUE;
  }
  else
    printf("Device with name %s NOT found \n",devName);
  return FALSE;
}

int psp_select_target_to_view_by_name(char *devName)
{

  if(!psp_select_object_to_view_by_name(devName))
  {
    pp3d_rob r =  psp_select_robot_by_name(devName);
    if(r==NULL)
    {
      printf("Neither a robot %s was found\n",devName);
      return FALSE;
    }
    p3d_select_robot_to_view(r);
  }

  return TRUE;

}

int psp_is_a_human(p3d_rob *r)
{

  if (strcasestr(r->name,"HUMAN"))
    return TRUE;
  return FALSE;
}


/************************************************************/
/*************** Link with task planners ********************/
/************************************************************/
/************************************************************/

static double psp_get_point_utility(double quality, double cost)
{

  double alfa = 0.5;
  double beta = 1-alfa;

  return ((alfa*quality)/100) + (1- ((beta*cost)/100));


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



/*
 double psl_srch_model_pt(p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method ,
 double viewPercent, hri_bitmapset* PSP_BTSET)
 {

 p3d_vector4 point, point2, v_aux;
 p3d_jnt *jntPt = objRob->joints[1];

 double refHumAngle, refHumAngle2, refHumAngle3;
 double x,y,xo,yo,zo;
 configPt qcurr, qaux,objqcurr, taskqcurr;
 int i, res, *vIndex,resTask=-1;
 int besti, response;
 float qQuality, *qUtils, maxUtil;
 //double kcd_with_report;
 FILE * ftaskCost =  fopen("taskcost.dat","w");
 int endLimit;
 qUtils = (float*) malloc(sizeof(float)*((numpoints+2)*numlayers));
 vIndex = (int*) malloc(sizeof(int)*((numpoints+2)*numlayers));
 testedX = (double*) malloc(sizeof(double)*((numpoints+2)*numlayers));
 testedY = (double*) malloc(sizeof(double)*((numpoints+2)*numlayers));

 ChronoOn();

 PSP_DRAW_QS = FALSE;

 destIndex = -1;
 qcurr     = p3d_get_robot_config(r);
 qaux      = p3d_get_robot_config(r);
 taskqcurr = p3d_get_robot_config(r);
 objqcurr  = p3d_get_robot_config(objRob);
 if (qcurr == NULL || qaux == NULL ||  objqcurr == NULL)
 {
 printf ("Impossible to allocate memory\n");
 return 0;
 }
 point[0] = qaux[ROBOTq_X];
 point[1] = qaux[ROBOTq_Y];
 point[2] = 0.0;
 point[3] = 1.0;

 v_aux[0] = qaux[ROBOTq_X] - objqcurr[HUMANq_X];
 v_aux[1] = qaux[ROBOTq_Y] - objqcurr[HUMANq_Y];
 v_aux[2] = 0.0;


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
 if (search_method[PSP_SRCHM_METHOD] == PSP_FRONT  && psp_is_a_human(objRob))
 oz = point2[2] = zo + (objRob->joints[55]->abs_pos[2][3]-zo)/1.5;
 else
 oz = point2[2];
 printf("z -> %f \n",zo);


 point[0] = 0.0;
 point[1] = 0.0;
 point[2] = 0.0;
 point[3] = 1.0;

 v_aux[0] = 0.0;
 v_aux[1] = 0.0;
 v_aux[2] = 0.0;

 qindex   = 0;
 testedQs = 0;

 //lastAngleCam  = qaux[ROBOTq_PAN];
 InitWaveCells(r->env->box.x1,r->env->box.y1,r->env->box.x2,r->env->box.y2, qaux[ROBOTq_X], qaux[ROBOTq_Y],PSP_BTSET);

 printGridVals();
 //printObstacles();
 //printGridObst();

 //jntPt2 = p3d_robot_dof_to_jnt(r,ROBOTq_PAN,&i_DoF);
 psp_gen_points(search_method[PSP_SRCHM_METHOD], objRob, r, numpoints, numlayers, &lstvert, PSP_BTSET);

 printf("List generated\n");

 theqs = (double**) realloc(theqs,sizeof(configPt*)*lstvert.nv);

 if (qindAnt<lstvert.nv)
 {
 for (i=qindAnt;i<lstvert.nv;i++)
 {
 theqs = p3d_get_robot_config(r);
 }
 }

 printf("memory allocated\n");

 //printListVtx(&lstvert);
 if (search_method[PSP_SRCHM_TYPE] == PSP_ORDERED)
 {
 printf("Search Type: ORDERED\n");
 psp_order_point_list(&lstvert);
 printf("List Ordered\n");
 }
 if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM ||search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
 srand((unsigned)time(0));


 p3d_set_and_update_this_robot_conf(r,qcurr);


 if (search_method[PSP_SRCHM_GOAL] ==  PSP_DEEP)
 endLimit = PSP_STOP_DEEP;
 else
 endLimit = lstvert.nv;

 for (i=0;i<endLimit;i++)
 {

 if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM)
 res = psp_get_next_random_point(point, numlayers, numpoints, &lstvert);
 else
 {
 if (search_method[PSP_SRCHM_TYPE] == PSP_RANDOM_LIST)
 res = psp_get_next_random_point_list(point, numlayers, numpoints, &lstvert);
 else
 res = psp_get_next_ordered_point(point,&lstvert);
 }

 if(res)
 {
 printf("Point generated %i - %f,%f\n",i,point[0],point[1]);
 // passing from local human  to global coords
 if (search_method[PSP_SRCHM_METHOD] == PSP_FRONT)
 p3d_matvec4Mult(jntPt->abs_pos,point,v_aux);
 else
 {
 v_aux[0] = point[0] +  point2[0];
 v_aux[1] = point[1] +  point2[1];
 }

 x = v_aux[0];
 y = v_aux[1];

 ///////////
 /// Modifying Robot postion and its camera angle
 ///////////
 qaux[ROBOTq_X] = x;
 qaux[ROBOTq_Y] = y;


 ///Horizontal robot angle
 refHumAngle = rad_angleOf(x,y,xo,yo);

 refHumAngle3 = rad_angleOf(xo,yo,x,y);
 //        lastAngle =  rad_angleOf(qaux[ROBOTq_X],qaux[ROBOTq_Y],objqcurr[HUMANq_X],objqcurr[HUMANq_Y]);
 // 	      if (lastAngle>2*M_PI)
 // 		lastAngle -=2*M_PI;
 //printf("Angles %f --- %f \n",refHumAngle2,refHumAngle3);
 //if (zo < 0.65 && linearDistance(x,y,xo,yo)<2)//check this conditional
 //  {
 //  if (refHumAngle3>3*(M_PI/2) && refHumAngle2<M_PI/2)
 //  {
 //  refHumAngle3 -= 2*M_PI;
 //  }

 //  if ( refHumAngle3 - refHumAngle2 < 0 )
 //  refHumAngle -= .7;
 //  else
 //  refHumAngle += .7;
 //  }

 /*
 //printf("Angle  %f max %f\n", refHumAngle,f_max);
 qaux[ROBOTq_RZ] = angleLim(refHumAngle);
 //printf("angle limited %f \n",qaux[ROBOTq_RZ]);
 //p3d_copy_config_into(r,qaux, &theqs[qindex]);

 //printf("Angles the qs %f --- %f \n",qaux[ROBOTq_RZ],theqs[qindex][ROBOTq_RZ]);
 //qindex++;
 //	      p3d_set_and_update_this_robot_conf(r,qaux);
 //	      g3d_refresh_allwin_active();

 qQuality = psp_test_qs(r, qaux, qcurr,point2,viewPercent,0, PSP_BTSET);
 //testedX[testedQs] = x;
 //testedY[testedQs] = y;
 //Qcosts[testedQs] = qQuality;
 //vIndex[testedQs] = lstvert.currentVert;
 testedQs++;
 //qcost = psp_test_qs(r, qaux, qcurr,point2,viewPercent,1, PSP_BTSET);
 if(qQuality > -1)
 {
 //if task is give object
 double taskValue,bestTaskVal;

 if (PSP_NEXT_TASK != PSP_NO_TASK)
 {

 p3d_copy_config_into(r,qaux,&taskqcurr);
 //sphereActive =1;

 if (psp_is_a_human(objRob))
 resTask = psp_give_to(r,objRob, &taskqcurr,&taskValue,&taskCost);
 else
 resTask = psp_take_from_surface(r,objRob->o[0], &taskqcurr,&taskValue);


 if (resTask)
 {
 printf("YES %f\n",taskValue);
 }
 else
 {
 printf("NO %f\n",taskValue);
 }
 }
 else
 {
 printf("NO NEXT TASK\n");
 taskValue = 0;
 resTask = 1;
 }
 if (taskValue>-1 && resTask)
 {
 //if(!p3d_col_test_robot(r,kcd_with_report))
 if (search_method[PSP_SRCHM_GOAL] ==  PSP_FFFO)
 {

 p3d_set_and_update_this_robot_conf(r,qcurr);
 p3d_destroy_config(r,objqcurr);
 g3d_draw_allwin_active();
 //PSP_DRAW_QS = TRUE;
 ChronoPrint("PSP - TIME");
 ChronoOff();
 //printListVtx(&lstvert);
 destIndex = i;
 p3d_destroy_config(r,qaux);
 p3d_destroy_config(r,qcurr);
 p3d_destroy_config(r,taskqcurr);
 free(qUtils);
 free(vIndex);
 return TRUE;
 }
 else
 {
 //theqs[qindex]  = realloc(theqs[qindex], sizeof());
 //p3d_copy_config_into(r,qaux, &theqs[qindex]);

 if (PSP_NEXT_TASK != PSP_NO_TASK)
 theqs[qindex]  =  p3d_copy_config(r,taskqcurr);
 else
 theqs[qindex]  =  p3d_copy_config(r,qaux);

 if (search_method[PSP_SRCHM_TYPE] != PSP_RANDOM)
 {
 lstvert.grid[lstvert.vertex[lstvert.currentVert].segment][lstvert.vertex[lstvert.currentVert].layer].obsPercent = qQuality;
 vIndex[qindex] = lstvert.currentVert;
 }
 else
 {
 //modify grid and list values
 }
 qUtils[qindex] = qQuality;

 if (qindex == 0)
 {
 besti = qindex;
 maxUtil = qQuality;
 bestTaskVal = taskValue;
 }
 else
 {
 if (PSP_NEXT_TASK != PSP_NO_TASK)
 {
 //if (psp_get_point_utility(qQuality, lstvert.vertex[lstvert.currentVert].cost )
 if (abs(qQuality-maxUtil)<5 && taskValue<bestTaskVal)
 {
 besti   = qindex;
 maxUtil = qQuality;
 bestTaskVal = taskValue;
 }
 }
 else
 {
 if(qQuality>maxUtil)
 {
 besti   = qindex;
 maxUtil = qQuality;
 //bestTaskVal = taskValue;
 }
 }
 }
 qindex++;
 }//end else  search_method
 }//task value
 fprintf(ftaskCost,"%f %f %f \n",x,y,taskValue);
 }
 else
 lstvert.vertex[lstvert.currentVert].status =  PSP_St_HIDDEN;
 }
 // else
 //	printf("Point %i out of range\n",i);
 }

 p3d_set_and_update_this_robot_conf(r,qcurr);

 if (qindex==0)
 {
 printf("---- ERROR: Not modeling point found -----\n");
 p3d_copy_config_into(r,qcurr, &(r->ROBOT_GOTO));
 p3d_copy_config_into(r,qcurr, &(r->ROBOT_POS));
 response = FALSE;
 }
 else
 {
 printf("---- %i Configurations  Generated -----\n",lstvert.nv);
 printf("---- %i Configurations  Tested    -----\n",testedQs);
 printf("---- %i Configurations  Found     -----\n",qindex);
 printf("---- Best configuration  on %i with %f in %f %f  -----\n", besti, maxUtil,theqs[besti][ROBOTq_X],theqs[besti][ROBOTq_Y]);
 //printQcosts(vIndex,Qcosts,qindex,&lstvert);
 //wey escoje el mejor qcost y  psp_local_grad_max
 // es mejor hacer una funcion que busque el maximo (o los) dentro de las configuraciones encontradas
 p3d_set_and_update_this_robot_conf(r,theqs[besti]);
 point[0] = theqs[besti][ROBOTq_X];
 point[1] = theqs[besti][ROBOTq_Y];
 point[2] = 0.0;
 point[3] = 1.0;
 destIndex = besti;
 printf("---- Local grad -----\n ");
 //psp_local_grad_max(point,v_aux, r, viewPercent, objRob, Qcosts[besti], theqs[besti], qaux, PSP_BTSET);

 printf("---- finnish -----\n ");

 p3d_copy_config_into(r,theqs[besti], &(r->ROBOT_GOTO));
 p3d_set_and_update_this_robot_conf(r,qcurr);

 //PSP_DRAW_QS = TRUE;
 // for (i=0;i<testedQs;i++)
 //{
 //printf("%f %f\n",theqs[i][ROBOTq_X],theqs[i][ROBOTq_Y]);
 //	  printf("%f %f\n",testedX[i],testedY[i]);
 //	  }
 printListVtx(&lstvert);
 response = TRUE;

 }
 fclose(ftaskCost);
 printf("freeing...\n");
 free(qUtils);
 free(vIndex);
 printf("freeing 2...\n");
 //free(testedX);
 //free(testedY);
 p3d_destroy_config(r,qaux);
 p3d_destroy_config(r,qcurr);
 p3d_destroy_config(r,objqcurr);
 printf("all free...\n");
 g3d_refresh_allwin_active();
 ChronoPrint("PSP - TIME");
 ChronoOff();
 //g3d_end_movie();
 //PSP_DRAW_QS = TRUE;
 return response;
 }


 */





/* Task aquisition */
static double psp_get_task_utility(char *task)
{
  char* taskpt;
  printf("task %s\n", task);
  taskpt = strtok(task,"(,)");
  while(taskpt != NULL)
  {
    printf(" ---> %s \n",taskpt);
    taskpt = strtok(NULL,"(,)");
  }
  return 0;
}

static int psp_update_world_state(char *task)
{

  printf("updating %s\n", task);

  return 0;
}

double psp_get_plan_utility(char *plan)
{

  char * strPlan = (char*)  malloc(sizeof(char)*strlen(plan));
  char* strpt;


  strcpy(strPlan,plan); //this is to avoid segfault on working with strings and constants


  printf("****** Analizing %s ********\n",plan);

  strpt = strtok(strPlan,":");

  if (strcasestr(strpt,"UPDATE"))
  {
    strpt = strtok(NULL,"/;");
    while(strpt != NULL)
    {
      psp_update_world_state(strpt);
      strpt = strtok(NULL,"/;");
    }
  }
  else
  {
    if (strcasestr(strpt,"PLAN"))
    {
      strpt = strtok(NULL,"/;");
      while(strpt != NULL)
	    {
	      psp_get_task_utility(strpt);
	      strpt = strtok(NULL,"/;");
	    }
    }
  }

  printf("****** End %s ********\n",plan);

  free(strPlan);
  return 0.0;
}



/***** Visibility tests ********/


int psp_is_object_visible(p3d_rob * robot, p3d_rob * object, double threshold)
{
  PSP_ROBOT = robot;
  p3d_select_robot_to_view(object);

  if (pso_watch3_obj() >= threshold)
  {
    p3d_deselect_robot_to_view(object);
    return TRUE;
  }
  p3d_deselect_robot_to_view(object);
  return FALSE;

}


int psp_is_body_visible(p3d_rob * robot, p3d_obj * object, double threshold)
{
  PSP_ROBOT = robot;

  object->caption_selected = 1;

  if (pso_watch3_obj() >= threshold)
  {
    object->caption_selected = 0;
    return TRUE;
  }
  object->caption_selected = 0;
  return FALSE;

}

int psp_seen_objects(p3d_rob* robot,  p3d_rob** list_of_seen_objects, double threshold)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  //p3d_vector4 objCenter,robCenter;
  int i;
  int nr = envPt->nr;
  //p3d_obj *o;
  p3d_rob *r;
  int contObj = 0;

  for(i=0;i<nr;i++)
  {
    r = envPt->robot[i];
    if( psp_is_object_visible(robot,  r,  threshold))
    {
      list_of_seen_objects[contObj] = r;
      contObj++;
    }
  }
  return contObj;
}


static void psu_reboot_theqs(p3d_rob *r, int numqs)
{
  int i;
  if (qindex > 0)
  {
    for (i=0;i<qindex;i++)
    {
      p3d_destroy_config(r,theqs[i]);
    }
  }
  theqs = (double**) realloc(theqs,sizeof(configPt*)*numqs);
  qindex=0;
}

static int psp_is_point_in_perspective_fov(p3d_vector4 p)
{
  int plan;
  G3D_Window *win = g3d_get_win_by_name((char *)"Perspective");
  g3d_refresh_win(win);

  for(plan = 0; plan < 6; plan++ ) // for all perspective window frustum plans
  {
    // if the point is in the negative side of the frustum plan means that it i maybe inside the frustum box
    if(win->frustum[plan][0] * (p[0]) + win->frustum[plan][1] * (p[1])
       + win->frustum[plan][2] * (p[2]) + win->frustum[plan][3] > 0 ) continue;
    return 0;
  }
  return 1;
}



int psp_is_object_in_fov(p3d_rob* robot, p3d_rob* object, double angleH, double angleW)
{

  p3d_rob* rtemp = PSP_ROBOT;
  PSP_ROBOT = robot;
  p3d_vector4 objectCenter;
  double tempAngH = robot->cam_h_angle;
  double tempAngW = robot->cam_h_angle;

  p3d_get_robot_center(object, objectCenter);
  robot->cam_h_angle = angleH;
  robot->cam_v_angle = angleW;

  if (psp_is_point_in_perspective_fov(objectCenter))
  {
    robot->cam_h_angle = tempAngH;
    robot->cam_v_angle = tempAngW;
    PSP_ROBOT = rtemp;
    return TRUE;
  }
  robot->cam_h_angle = tempAngH;
  robot->cam_v_angle = tempAngW;
  PSP_ROBOT = rtemp;

  return FALSE;
}

int psp_is_body_in_fov(p3d_rob* robot, p3d_obj* object, double angleH, double angleW)
{
  p3d_rob* rtemp = PSP_ROBOT;
  PSP_ROBOT = robot;
  p3d_vector4 objectCenter;
  double tempAngH = angleH;
  double tempAngW = angleW;

  p3d_get_object_center(object, objectCenter);
  ox =  objectCenter[0];
  oy =  objectCenter[1];
  oz =  objectCenter[2];
  sphereActive =1;
  robot->cam_h_angle = angleH;
  robot->cam_v_angle = angleW;

  if (psp_is_point_in_perspective_fov(objectCenter))
  {
    robot->cam_h_angle = tempAngH;
    robot->cam_v_angle = tempAngW;
    PSP_ROBOT = rtemp;
    return TRUE;
  }
  robot->cam_h_angle = tempAngH;
  robot->cam_v_angle = tempAngW;
  PSP_ROBOT = rtemp;
  return FALSE;
}

void psp_draw_random_points(p3d_rob* robot)
{
  lstvert.nv=300;
  p3d_vector4 objectCenter, point;
  p3d_rob* rtemp = PSP_ROBOT;
  PSP_ROBOT = robot;

  p3d_get_robot_center(robot, objectCenter);
  int i;

  for (i=0; i<300; i++) {
    psp_gen_rand_3Dpoint(point, objectCenter, 0, 10);
    if (psp_is_point_in_perspective_fov(point))
    {
      lstvert.vertex[i].pos[0] = point[0];
      lstvert.vertex[i].pos[1] = point[1];
      lstvert.vertex[i].pos[2] = point[2];
      lstvert.vertex[i].status = PSP_St_OBSERVABLE;
    }
    else
    {
      lstvert.vertex[i].pos[0] = 0;
      lstvert.vertex[i].pos[1] = 0;
      lstvert.vertex[i].pos[2] = 0;
      lstvert.vertex[i].status = PSP_St_NOT_IN_RANGE;
    }
  }

  PSP_ROBOT = rtemp;
}



int psu_get_num_objects_in_fov(p3d_rob *currRob, double radius, int type, double limDist, p3d_obj **oList, double *distances)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 objCenter,robCenter;
  int i,j;
  int no = envPt->no;
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int contObj =0;
  p3d_vector4 pointHead, pointAhead;
  double disttocenter;
  //p3d_rob *human = PSP_BTSET->human[PSP_BTSET->actual_human]->HumanPt;

  p3d_get_robot_center(currRob, robCenter);

  //Part that draws the line in the center of the fov

  psu_get_point_ahead_cam(currRob, radius, pointAhead);
  p3d_get_object_center(currRob->o[currRob->cam_body_index], pointHead);

  lx1 = pointHead [0];
  ly1 = pointHead [1];
  lz1 = pointHead [2];

  lx2 = pointAhead [0];
  ly2 = pointAhead [1];
  lz2 = pointAhead [2];
  // until here

  psp_deselect_all();

  //Static Obstacles
  if (type==0 || type==2)
    for(i=0;i<no;i++)
    {
      o = envPt->o[i];
      if (!strcasestr(o->name,"furn"))
      {
        p3d_get_object_center(o,objCenter);

        if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	      {
          if (psp_is_body_in_fov(currRob, o, 0.7, 0.7))  //(objCenter, pointHead, pointAhead, 0.7, &disttocenter))
          {
            PSP_DRAW_OBJ_ARRAY [contObj] = i;
            oList[contObj] = o;
            distances [contObj] = p3d_psp_pointtolinedist(objCenter,pointHead,pointAhead);
            contObj++;
          }
	      }
      }
    }

  //Robot body parts
  if (type==1 || type ==2)
    for(i=0;i<nr;i++)
    {
      r = envPt->robot[i];
      for(j=0;j<r->no;j++)
      {
        o = r->o[j];
        //if (strasestr(o->name,"head") || strasecstr(o->name,"hand") c)
        // {
        p3d_get_object_center(o,objCenter);
        if(linearDistance(robCenter[0],robCenter[1], objCenter[0], objCenter[1])<=radius)
	      {
          if ( psp_is_body_in_fov(currRob, o, 0.7, 0.7) )  //p3d_psp_is_point_in_a_cone(objCenter, pointHead, pointAhead, 1.0, &disttocenter))
          {
            if(disttocenter <= limDist)
            {
              oList[contObj] = o;
              contObj++;
            }
          }
	      }
        // }
      }


    }

  return contObj;
}
