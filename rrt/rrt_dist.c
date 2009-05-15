#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
//#include "Collision-pkg.h"
#include "Rrt-pkg.h"

#define sqr(x) ((x)*(x))

/* les fonction locales */
static double dist_axe(double *q1, double *q2);
static double user_dist (double *q1,double *q2);
static double max_moving_dist (double *q1, double *q2);
static double dist_mes (double *q1, double *q2);
static double dist_length(p3d_rob *r, double *q_start, double *q_goal);


/************************************************/
/* print the distance between q init and q goal */
/************************************************/

void print_distance(double *q1,double *q2,int dist_choice)
{
  double dist;
  double *tab;
  int nddl = p3d_get_robot_njnt();                         /* number of dof */
  tab = MY_ALLOC(double, nddl+1);

  if (dist_choice == 5)
    dist_jnt_axe(tab);

  dist = distance_rrt(q1,q2,dist_choice);
  PrintInfo(("distance q init / q goal: %f\n", dist));
  MY_FREE(tab, double,nddl+1);
}


/*****************************************/
/* appele a l'initialisation             */
/* calcul de la distance max du corps    */
/* a l'axe de rotation pour chaque joint */
/*****************************************/

void dist_jnt_axe(double *tab)
{ 
}



/*****************************************/
/* calcul env distance divise par le pas */
/*****************************************/


double calcul_env_dist(int dist_choice, int pas)
{
  int nddl = p3d_get_robot_njnt();               
  int nb_dof = p3d_get_robot_ndof();
  double env_dist;
  double *min = MY_ALLOC (double, nb_dof);
  double *max = MY_ALLOC (double, nb_dof);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  int i, j, k;
  double step;
  p3d_jnt * jntPt;
  
  step = ((double)pas)/100.0;

  /*trouver les config de la boite */
  for (i=0;i<=nddl;i++) {
    jntPt = r->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      p3d_jnt_get_dof_bounds(jntPt, j, &(min[i]), &(max[i]));
    }
  }

  env_dist = distance_rrt (min, max, dist_choice) * step;
  MY_FREE(min, double, nb_dof);
  MY_FREE(max, double, nb_dof);
  if (dist_choice == 4)
    env_dist = distance_rrt (min, max, 1);
  return (env_dist);
}




/**************************************************************/
/* the next function choice the distance function *************/
/**************************************************************/

double distance_rrt (double *q1, double *q2, int dist_choice)
{
  double dist = 0.;
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */

  switch (dist_choice)
    {
    case 1:
      dist = dist_mes(q1, q2);
      break;
    case 2:
      dist = max_moving_dist(q1, q2);
      break;
    case 3:
      dist = dist_length (r, q1, q2);
      break;
    case 4:
      dist = user_dist(q1, q2);
      break;
    case 5:
      dist = dist_axe(q1, q2);
      break;
    default:
      PrintInfo(("erreur dans le choix de la fonction distance\n"));
    } 

  return (dist);
}


/********************************************/
/* Calcul de la distance maximale parcourue */
/* le robot ou le dernier joints du robot   */
/********************************************/

static double
max_moving_dist (double *q1, double *q2)
{
  int njnt = p3d_get_robot_njnt();                         /* number of dof */
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  double Dmax,*D;
  int ij;
  p3d_jnt **array_jnt;
  p3d_vector4 *coor1, *coor2;
  p3d_vector4 coor_rob;

  array_jnt = r->joints;
  D = MY_ALLOC(double,(njnt+1));    
  coor1 = MY_ALLOC(p3d_vector4,2);
  coor2 = MY_ALLOC(p3d_vector4,2);
  Dmax = 0.; 
 
  p3d_set_and_update_robot_conf(q1);
  coor_rob[3] = 1;
  coor1[0][0] = array_jnt[0]->abs_pos[0][3];
  coor1[0][1] = array_jnt[0]->abs_pos[1][3];  
  coor1[0][2] = array_jnt[0]->abs_pos[2][3];
  coor_rob[0] = array_jnt[njnt]->abs_pos[0][3];
  coor_rob[1] = array_jnt[njnt]->abs_pos[1][3];
  coor_rob[2] = array_jnt[njnt]->abs_pos[2][3];
  
  p3d_set_and_update_robot_conf(q2);
  coor2[0][0] = array_jnt[0]->abs_pos[0][3];
  coor2[0][1] = array_jnt[0]->abs_pos[1][3];  
  coor2[0][2] = array_jnt[0]->abs_pos[2][3];
  coor_rob[0] = array_jnt[njnt]->abs_pos[0][3];
  coor_rob[1] = array_jnt[njnt]->abs_pos[1][3];
  coor_rob[2] = array_jnt[njnt]->abs_pos[2][3];

  for(ij=0;ij<2;ij++)
    {
      D[ij] = sqrt(sqr(coor1[ij][0]-coor2[ij][0])+sqr(coor1[ij][1]-coor2[ij][1])+sqr(coor1[ij][2]-coor2[ij][2]));
      //      PrintInfo(("D[%d] = %f\n",ij,D[ij]));
      if(D[ij]!= 0.)
	if(D[ij]>Dmax)
	  Dmax = D[ij];
    }

  MY_FREE(D,double,(njnt+1));    
  MY_FREE(coor1,p3d_vector4,(njnt+1));
  MY_FREE(coor2,p3d_vector4,(njnt+1));
  return (Dmax);
}


/**********************************/
/* calcul approche de la distance */
/* max parcourue entre 2 config   */
/**********************************/

static double
dist_axe(double *q1, double *q2)
{
  int nddl = p3d_get_robot_njnt();                        /* number of joint */
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot   */
  p3d_jnt *jnt;
  double dist;
  double delta;
  int i, j, k;

  dist = dist_mes(q1,q2);

  for (i=0;i<=nddl;i++) {
    jnt = r->joints[i];	
    for(j=0; j<jnt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jnt, j)) {
	k = jnt->index_dof+j;
	delta = fabs(q2[k]-q1[k]);
	dist = dist + (delta*(3.14159/180)*jnt->dist);
      }
    }
  }
  return (dist);
}

/*****************************************************************************************/
/* the next function calculate the distance between 2 configs considering the max moving */
/*****************************************************************************************/

static double
user_dist (double *q1,double *q2)
{
  int nddl = p3d_get_robot_njnt();
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  int i, j, k;
  double min,max,dist=0.;
  p3d_jnt * jntPt;

  for (i=0;i<=nddl;i++) {
    jntPt = r->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof;
      p3d_jnt_get_dof_bounds(jntPt, j, &min, &max);
      if (max>min+EPS6) {
	if (p3d_jnt_is_dof_circular(jntPt, j))
	  { dist = MAX(ABS(angle_limit_PI(q1[i]-q2[i])/(2*M_PI)), dist); }
	else
	  { dist = MAX(ABS((q1[i]-q2[i])/(max-min)),dist); }
      }
    }
  }

  return(dist);
}

/*************************************************************************/
/* the next function calculate the distance between 2 configs */
/* considering the space distance */
/*************************************************************************/
/*
  calcul la distance euclidienne dans (x,y,z)
*/

static double
dist_mes (double *q1, double *q2)
{
  int i;
  double dist;
  double coor[3];

  for (i=0; i<NDOF_BASE_TRANSLATE; i++)
      coor[i] = (q1[i]-q2[i])*(q1[i]-q2[i]);
  dist = sqrt (coor[0] + coor[1] + coor[2]);
  return (dist);
}

static double
dist_length(p3d_rob *r, double *q_start, double *q_goal)
{
  pp3d_localpath c = NULL;

  c = p3d_local_planner(r, q_start, q_goal);
  
  if(c == NULL)
    {
      PrintError(("localpath NULL !!!\n"));
      return FALSE;		/* chemin non valide */
    }

  return(c->length(r,c));

}
