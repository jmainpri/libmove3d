/*! \file kcd_api_report.c
\brief access to different reports

option with_report

- NO_REPORT 
  nothing is written in a report
- JUST_BOOL 
  only the fact of given group of movable object 
  is in collision or not is written in a report
- DISTANCE_ESTIMATE 
  Estimated distances are written in a report
  The following four reports contain the information
  kcd_distance_report_table[group_id][mo_id_in_group]
  kcd_dist_mo_report_table[mo_id]
  for objects belonging to the same group
    kcd_distance_auto_report_table_mos[mo_id]
  for objects belonging to different group
    kcd_distance_report_table_mos[mo_id]
- DISTANCE_EXACT
  Exact distances are written in a report, the corresponding
  closest points are also written in the report. These points and
  distances are valid if the no collisions are detected
  There is a difference made by the distances till the static objects
  or the distance till the moving objects
  *-*  mo vs so  *-*
    contains the distances till the static objects 
    for objects belonging to a group
      kcd_distance_report_table[group_id][mo_id_in_group]
    for objects not belonging to a group
      kcd_dist_mo_report_table[mo_id]
  *-* mo vs mo *-* 
    contains the distances till the mobile objects 
      kcd_distance_report_table_mos[mo_id]
  
  If the relative error parameter is set to a value between
  0 and 100 then the reports will contain the safest distance
  which will be [ d - (d * (error/100) ) ]

*/

#include "Util-pkg.h"
#include "Collision-pkg.h"

/* B Kineo Carl 22.02.2002 */
typedef struct kcd_colliding_pair
{
  void *primId1; /*  first polyhedron or solid (always movable)    */
  int ext_prim_id1;
  void *primId2; /* second polyhedron or solid (movable or static) */
  int ext_prim_id2;
}t_kcd_colliding_pair, *t_kcd_colliding_pair_p;
/* E Kineo Carl 22.02.2002 */

/* typedef double *doublep; */
/* structure used for the closest points between 
 * each movable object and the static environment
 */

typedef struct kcd_closest_pair_struct
{
  kcd_vector3 point_mo; /* movable object */
  kcd_vector3 point_so; /* static object */
  double min_distance;
}kcd_closest_pair_struct, *kcd_closest_pair_struct_p;

/* structure used for the closest points between
 * the different movable objects
 */

typedef struct kcd_closest_pair_struct_mos
{
  kcd_vector3 point_mo1; /* movable object 1 */
  kcd_vector3 point_mo2; /* movable object 2 */
  double min_distance;
  int in_collision_with_mo_id;
}kcd_closest_pair_struct_mos, *kcd_closest_pair_struct_mos_p;

/* variables for nearest obstacle and nr collisions */
static int nr_kcd_collisions = 0;
int *p3d_kcd_nearest_obstacle = NULL;
int *p3d_kcd_report = NULL;

/* these are the old report variables 
   static doublep *kcd_distance_report_table = NULL;
   static double  *kcd_dist_mo_report_table  = NULL;*/

/* mo vs so */
static kcd_closest_pair_struct_p *kcd_distance_report_table = NULL;
static kcd_closest_pair_struct   *kcd_dist_mo_report_table  = NULL;
/* mo vs mo */
static kcd_closest_pair_struct_mos *kcd_distance_report_table_mos = NULL;

/* B Kineo Carl 22.02.2002 */
/* for each movable object, 
   the pair of polyh/solids between which a collision was found */
static t_kcd_colliding_pair *kcd_pairInCollision = NULL;
/* E Kineo Carl 22.02.2002 */

/* B Kineo Carl 22.02.2002 */
/** ***************************************************************** **
 ** ***************************************************************** **
 ** Functions: keep track of polyhedrons and solids found to be 
 **            in collision at the latest test
 ** ***************************************************************** **
 ** ***************************************************************** **/
void kcd_init_pairInCollision()
{
  int i;
  kcd_pairInCollision = NULL;
  kcd_pairInCollision = MY_ALLOC(t_kcd_colliding_pair,1);
  i = 0;
  kcd_pairInCollision[i].primId1 = NULL;
  kcd_pairInCollision[i].ext_prim_id1 = -1;
  kcd_pairInCollision[i].primId2 = NULL;
  kcd_pairInCollision[i].ext_prim_id2 = -1;
}

void kcd_get_pairInCollision(void **primId1, void **primId2)
{
  if((kcd_pairInCollision != NULL)     )        /*   && (moId >= 0)    && (moId < maxMos) */
    {
      if((kcd_pairInCollision[0].primId1 != NULL) && (kcd_pairInCollision[0].primId2 != NULL))
	{
	  *primId1 = kcd_pairInCollision[0].primId1;
	  *primId2 = kcd_pairInCollision[0].primId2;
	}
    }
}

void kcd_get_pairObjInCollision(int *objId1, int *objId2)
{
  int p1 = -1;
  int p2 = -1;

  if((kcd_pairInCollision != NULL)     )        /*   && (moId >= 0)    && (moId < maxMos) */
    {
      if((kcd_pairInCollision[0].primId1 != NULL) && (kcd_pairInCollision[0].primId2 != NULL))
	{
	  p1 = kcd_pairInCollision[0].ext_prim_id1;
	  p2 = kcd_pairInCollision[0].ext_prim_id2;
	  *objId1 = kcd_obj_prim_table[p1].ext_obj_id;  
	  *objId2 = kcd_obj_prim_table[p2].ext_obj_id;
	}
    }  
}

void kcd_set_pairInCollision(void *primId1, int ext_p_id1, void *primId2, int ext_p_id2)
{
  if((kcd_pairInCollision != NULL) 
     /*  && (moId >= 0)    && (moId < maxMos) */
     )
    {
      kcd_pairInCollision[0].primId1 = primId1;
      kcd_pairInCollision[0].ext_prim_id1 = ext_p_id1;
      kcd_pairInCollision[0].primId2 = primId2;
      kcd_pairInCollision[0].ext_prim_id2 = ext_p_id2;
    };
}

void kcd_clean_up_pairInCollision()
{
  if(kcd_pairInCollision)
    MY_FREE(kcd_pairInCollision,t_kcd_colliding_pair,1);
  kcd_pairInCollision = NULL;
}
/* E Kineo Carl 22.02.2002 */

/** ***************************************************************** **
 ** ***************************************************************** **
 ** Functions: keep track of distance estimate to nearest obstacle for 
 **            each of the movable objects, organized by group (robot) 
 ** ***************************************************************** **
 ** ***************************************************************** **/
void kcd_init_dist_mo_report_table(int nof_mos)
{
  /*int i;*/
  kcd_dist_mo_report_table = NULL;
  kcd_dist_mo_report_table = MY_ALLOC(kcd_closest_pair_struct,nof_mos);
 /* for(i=0; i<nof_mos ; i++)
    kcd_dist_mo_report_table[i].min_distance = P3D_HUGE;*/
}


/* ******************************************************************* *
 * Function:  kcd_get_dist_mo_so()
 *            returns for given movable object the distance estimate 
 *            stored in the kcd_dist_mo_report_table
 * ARGS IN : mo_id                   the movable object
 * ARGS OUT: d                       the distance estimate
 * PRE_COND: kcd_init_kcd_dist_mo_report_table() called before
 * ******************************************************************* */
void kcd_get_dist_mo_so(int mo_id,double *d)
{
  *d = kcd_dist_mo_report_table[mo_id].min_distance;
}
/* ******************************************************************* *
 * Function:  kcd_set_dist_mo_so() 
 *            sets for given movable object the given distance estimate 
 *            stored in the kcd_dist_mo_report_table
 * ARGS IN : mo_id                   the movable object
 *           d                       the distance estimate
 * PRE_COND: kcd_init_kcd_dist_mo_report_table() called before
 * ******************************************************************* */
void kcd_set_dist_mo_so(int mo_id,double d)
{
  kcd_dist_mo_report_table[mo_id].min_distance = d;
}
/* ******************************************************************* *
 * Function:  kcd_clean_up_dist_mo_report_table()
 *            cleans the distance report table away for the mov. objs.
 * ARGS IN :  nof_mos        the total number of movable objects
 * ******************************************************************* */
void kcd_clean_up_dist_mo_report_table(int nof_mos)
{
  if(kcd_dist_mo_report_table)
    MY_FREE(kcd_dist_mo_report_table,kcd_closest_pair_struct,nof_mos);
  kcd_dist_mo_report_table = NULL;
}

/** ***************************************************************** **
 ** ***************************************************************** **
 ** Functions: keep track of nearest AABB or OBB around static object 
 **            to each of the movable objects                            
 ** ***************************************************************** **
 ** ***************************************************************** **/

/* ******************************************************************* *
 * Function: kcd_init_nearest_obstacle()
 *           initializes global variable array p3d_kcd_nearest_obstacle
 * ******************************************************************* */
void kcd_init_nearest_obstacle()
{
  int i,nr_robots = 0;

  nr_robots = kcd_get_nof_grps();
  p3d_kcd_nearest_obstacle = NULL;
  p3d_kcd_nearest_obstacle = MY_ALLOC(int,nr_robots);
  for(i=0;i<nr_robots;i++)
    {
      p3d_kcd_nearest_obstacle[i] = -2;
    }
}

/* ******************************************************************* *
 * Function: kcd_modify_nearest_obstacle()
 *           sets nearest bb of a group of movable objects
 * ARGS IN : rob_i                id of a group of movable objects
 *           static_object_bb_id  id of nearest BB
 * ******************************************************************* */
void kcd_modify_nearest_obstacle(int rob_i, int static_object_bb_id)
{
  p3d_kcd_nearest_obstacle[rob_i] = static_object_bb_id;
}

/* ******************************************************************* *
 * Function: kcd_get_nearest_obstacle_bb_id()
 *           returns nearest bb of a group of movable objects
 * ARGS IN : rob_i                id of a group of movable objects
 * ARGS OUT:                      id of nearest BB (or -1 if error)
 * ******************************************************************* */
int kcd_get_nearest_obstacle_bb_id(int rob_nr)
{
  if((kcd_is_initialized()) && (p3d_kcd_nearest_obstacle != NULL))
    {
      return (p3d_kcd_nearest_obstacle[rob_nr]);
    }
  else
    return (-1);
}

/*!
  \brief for a given group this function retrieves the placement matrix of the
  nearest obstacle
  \param group_id the id of the group of movable objects(IN)
  \param SOwrtW the placement matrix of the static object (OUT) or x,y,z coordinates
  \return 0 then the information is not valid
  \return 1 then SOwrtW contains the coordinates od the AABB around the nearest obstacle,
             SOwrtW[0][0]->x1;	  
	     SOwrtW[0][1]->x2;
	     SOwrtW[1][0]->y1;	 
	     SOwrtW[1][1]->y2;
	     SOwrtW[2][0]->z1;	
	     SOwrtW[2][1]->z2;   
  \return 2 then SOwrt contains the placement matrix
  \warning works only with DISTANCE_ESTIMATE as with_report parameter
  \warning the returned matrix is only valid if the robot is close to the obstacles
*/
int kcd_get_nearest_obstacle_information(int group_id, kcd_matrix4 SOwrtW)
{
  int bb_id =  kcd_get_nearest_obstacle_bb_id(group_id);
  if(bb_id < 0)
    {
      return 0;
    }
  else if(all_bbs[bb_id]->entity_type == KCD_AABB)
    {
      SOwrtW[0][0] = all_bbs[bb_id]->x1;	  
      SOwrtW[0][1] = all_bbs[bb_id]->x2;
      SOwrtW[1][0] = all_bbs[bb_id]->y1;	 
      SOwrtW[1][1] = all_bbs[bb_id]->y2;
      SOwrtW[2][0] = all_bbs[bb_id]->z1;	
      SOwrtW[2][1] = all_bbs[bb_id]->z2;   
      return 1;
    }
  else
    {
      SOwrtW[0][0] = all_bbs[bb_id]->eigenv[0][0];	  
      SOwrtW[1][0] = all_bbs[bb_id]->eigenv[0][1];
      SOwrtW[2][0] = all_bbs[bb_id]->eigenv[0][2];	 
      SOwrtW[0][3] = all_bbs[bb_id]->center[0];
      SOwrtW[0][1] = all_bbs[bb_id]->eigenv[1][0];	
      SOwrtW[1][1] = all_bbs[bb_id]->eigenv[1][1];
      SOwrtW[2][1] = all_bbs[bb_id]->eigenv[1][2];	
      SOwrtW[1][3] = all_bbs[bb_id]->center[1];
      SOwrtW[0][2] = all_bbs[bb_id]->eigenv[2][0];	
      SOwrtW[1][2] = all_bbs[bb_id]->eigenv[2][1];
      SOwrtW[2][2] = all_bbs[bb_id]->eigenv[2][2];	
      SOwrtW[2][3] = all_bbs[bb_id]->center[2];
      SOwrtW[3][0] = 0.0;	
      SOwrtW[3][1] = 0.0;
      SOwrtW[3][2] = 0.0;	                  
      SOwrtW[3][3] = 1.0;
      return 2;
    }
}


/* ***************************************************************** **
 ** ***************************************************************** **
 ** Functions: keep track of boolean answer of KCD for
 **            each of the groups of movable objects      
 ** ***************************************************************** **
 ** ***************************************************************** **/

/* ******************************************************************* *
 * Function: kcd_init_report()
 *           initializes global variable array p3d_kcd_report
 * ******************************************************************* */
void kcd_init_report()
{
  int i,nr_robots = 0;

  p3d_kcd_report = NULL;

  nr_robots = kcd_get_nof_grps();
  p3d_kcd_report = MY_ALLOC(int,nr_robots);  
  for(i=0;i<nr_robots;i++)
    {
      p3d_kcd_report[i] = FALSE;
    }
}
/* ******************************************************************* *
 * Function: kcd_clean_up_report()
 *           cleans global variable array p3d_kcd_report
 * ******************************************************************* */
void kcd_clean_up_report()
{
  int nr_robots;

  nr_robots = kcd_get_nof_grps();
  MY_FREE(p3d_kcd_report,int,nr_robots);
  p3d_kcd_report = NULL;
  MY_FREE(p3d_kcd_nearest_obstacle,int,nr_robots);
  p3d_kcd_nearest_obstacle = NULL;
}

/* ******************************************************************* *
 * Function: kcd_set_report_to_zero()
 *           sets each entry of array p3d_kcd_report to FALSE
 *           and nr_kcd_collisions to 0 (initially: no collisions found)
 * ******************************************************************* */
void kcd_set_report_to_zero()
{
  int i, nr_robots, nr_mobj;

  /* PrintInfo(("kcd_set_report_to_zero\n")); */
  nr_robots = kcd_get_nof_grps();
  nr_mobj = kcd_get_nof_mos();
 
  for(i=0;i<nr_robots;i++)
    {
      p3d_kcd_report[i] = FALSE;
    }
  nr_kcd_collisions = 0;

  /* reset the dist_mo_table */
  kcd_reset_dist_mo_report_table(nr_mobj);  

    /* reset the dist_so_table */
  kcd_reset_distance_report_table_grp_so();

}

/* ******************************************************************* *
 * Function: kcd_add_report() 
 *           sets p3d_kcd_report to TRUE for a given group of 
 *           movable objects
 * ARGS IN : rob_i                   the group of movable objects
 *           static_object_bb_id     the bb of a static object in 
 *                                   collision with the group, or -1 in 
 *                                   case of self-collision in the group
 * PRECOND:  kcd_init_report() called before 
 * REMARKS:  static_object_bb_id not stored, not used
 * ******************************************************************* */
void kcd_add_report(int rob_i,int static_object_bb_id)
{
  p3d_kcd_report[rob_i] = TRUE;
  nr_kcd_collisions++;
}

/* ******************************************************************* *
 * Function:  kcd_check_report() 
 *            returns for given group of movable objects 
 *            the corresponding boolean value in collision report 
 * ARGS IN : rob_i                   the group of movable objects
 * PRE_COND: kcd_init_report() called before
 * ******************************************************************* */
int kcd_check_report(int rob_nr)
{
  if(nr_kcd_collisions > 0)
    return (p3d_kcd_report[rob_nr]);
  else
    return FALSE;
}

/** ***************************************************************** **
 ** ***************************************************************** **
 ** Functions: keep track of answer with distance estimate of KCD for
 **            each of the movable objects                            
 ** ***************************************************************** **
 ** ***************************************************************** **/

/* ******************************************************************* *
 * Function:  kcd_get_dist_grp_mo_so()
 *            returns for given movable object in a given group of 
 *            movable objects the distance estimate 
 *            stored in the kcd_distance_report_table
 * ARGS IN : rob_i                   the group of movable objects
 *           body_id                 the movable object
 * PRE_COND: kcd_init_kcd_distance_report_table() called before
 * ******************************************************************* */
void kcd_get_dist_grp_mo_so(int robot_id,int body_id,double *d)
{
  *d = kcd_distance_report_table[robot_id][body_id].min_distance;
}

/* ******************************************************************* *
 * Function:  kcd_set_distance_body_obst() 
 *            sets for given movable object in a given group of 
 *            movable objects the given distance estimate 
 *            stored in the kcd_distance_report_table
 * ARGS IN : rob_i                   the group of movable objects
 *           body_id                 the movable object
 *           d                       the distance estimate
 * PRE_COND: kcd_init_kcd_distance_report_table() called before
 * ******************************************************************* */
void kcd_set_distance_body_obst(int robot_id,int body_id,double d)
{
  kcd_distance_report_table[robot_id][body_id].min_distance = d;
}
/* Modification Pepijn debut may 2001 */

/* ******************************************************************* *
 * Function:  kcd_set_points_closest_pair() 
 *            sets for given movable object in a given group of 
 *            movable objects the coordinates of the closest pair 
 *            of points
 *         
 * ARGS IN : rob_i                   the group of movable objects
 *           body_id                 the movable object
 *           point_mo                coordinate of the closest point 
 *                                   of the movable object
 *           point_so                coordinate of the closest point 
 *                                   of the static object
 * PRE_COND: kcd_init_kcd_distance_report_table() called before
 * ******************************************************************* */
void kcd_set_points_closest_pair(int robot_id,int body_id, 
				 kcd_vector3 point_mo, kcd_vector3 point_so)
{
  kcd_distance_report_table[robot_id][body_id].point_mo[0] = point_mo[0];
  kcd_distance_report_table[robot_id][body_id].point_mo[1] = point_mo[1];
  kcd_distance_report_table[robot_id][body_id].point_mo[2] = point_mo[2];

  kcd_distance_report_table[robot_id][body_id].point_so[0] = point_so[0];
  kcd_distance_report_table[robot_id][body_id].point_so[1] = point_so[1];
  kcd_distance_report_table[robot_id][body_id].point_so[2] = point_so[2];
}

/* ******************************************************************* *
 * Init kcd_distance_report_table
 * IN: nof_robots  -> nr of groups of movable objects
 * ******************************************************************* */
void kcd_init_kcd_distance_report_table(int nof_robots)
{
  int i;

  kcd_distance_report_table = NULL;
  kcd_distance_report_table = MY_ALLOC(kcd_closest_pair_struct_p,nof_robots);
  for(i=0;i<nof_robots;i++)
    kcd_distance_report_table[i] = NULL;
}


/* ******************************************************************* *
 * Function:  kcd_init_kcd_distance_report_table_entry() 
 *            initializes the distance estimate table entries
 *            for the given group
 *            stored in the kcd_distance_report_table
 * ARGS IN :  robot_id         id of the group of movable objects
 *            nof_bodies       the number of movable objects in 
 *                             the given group
 * ******************************************************************* */
void kcd_init_kcd_distance_report_table_entry(int robot_id,int nof_bodies)
{
  int i;

  kcd_distance_report_table[robot_id] = NULL;
  kcd_distance_report_table[robot_id] = MY_ALLOC(kcd_closest_pair_struct,nof_bodies);  
  for(i=0;i<nof_bodies;i++)
    {
    kcd_distance_report_table[robot_id][i].min_distance = P3D_HUGE;

    kcd_distance_report_table[robot_id][i].point_mo[0] = P3D_HUGE;
    kcd_distance_report_table[robot_id][i].point_mo[1] = P3D_HUGE;
    kcd_distance_report_table[robot_id][i].point_mo[2] = P3D_HUGE;
    
    kcd_distance_report_table[robot_id][i].point_so[0] = P3D_HUGE;
    kcd_distance_report_table[robot_id][i].point_so[1] = P3D_HUGE;
    kcd_distance_report_table[robot_id][i].point_so[2] = P3D_HUGE;
    }
}

/* ******************************************************************* *
 * Function:  kcd_init_distance_report_table()
 *            gets number of groups of mo-s in the scene, 
 *            for each group: gets number of mo-s in the group and 
 *            calls initialization function for distance report table
 * REMARKS :  should be made independent from user-data, dependent only
 *            of kcd-api when user defines groups
 * ******************************************************************* */
void kcd_init_distance_report_table()
{
  int nof_robots; 
  int nof_bodies,rit;

  nof_robots = kcd_get_nof_grps();

  kcd_init_kcd_distance_report_table(nof_robots);
  for(rit=0;rit<nof_robots;rit++)
    {
      nof_bodies = kcd_get_nof_grp_elts(rit);
      kcd_init_kcd_distance_report_table_entry(rit,nof_bodies);
    }
}

/* ******************************************************************* *
 * Function:  kcd_clean_up_kcd_distance_report_table_entry()
 *            cleans the distance report table entries
 *            away for a given group
 * ARGS IN :  robot_id         the id of a group of movable objects
 *            nof_bodies       the number of movable objects in the 
 *                             given group
 * ******************************************************************* */
void kcd_clean_up_kcd_distance_report_table_entry(int robot_id,int nof_bodies)
{
  if(kcd_distance_report_table[robot_id])
    {
      MY_FREE(kcd_distance_report_table[robot_id],kcd_closest_pair_struct,nof_bodies);
      kcd_distance_report_table[robot_id] = NULL;
    }  
}

/* ******************************************************************* *
 * Function:  kcd_clean_up_kcd_distance_report_table()
 *            cleans the distance report table away for a given group
 * ARGS IN :  robot_id         the id of a group of movable objects
 * PRECOND :  function kcd_clean_up_kcd_distance_report_table_entry()
              should have been called earlier
 * ******************************************************************* */
void kcd_clean_up_kcd_distance_report_table(int nof_robots)
{
  if(kcd_distance_report_table)
    MY_FREE(kcd_distance_report_table,kcd_closest_pair_struct_p,nof_robots);
  kcd_distance_report_table = NULL;
}


/* ******************************************************************* *
 * Function:  kcd_clean_up_distance_report_table()
 *            gets number of groups in the scene, the number of mo-s
 *            in each of the groups and
 *            cleans the whole distance report table away
 * REMARKS :  should depend only of kcd-api, not of user-data
 * ******************************************************************* */
void kcd_clean_up_distance_report_table()
{
  int nof_robots;
  int nof_bodies,rit;
  int nof_mos;

  nof_mos = kcd_get_nof_mos();
  nof_robots = kcd_get_nof_grps();
  for(rit=0;rit<nof_robots;rit++)
    {
      nof_bodies = kcd_get_nof_grp_elts(rit);
      kcd_clean_up_kcd_distance_report_table_entry(rit,nof_bodies);
    }
  kcd_clean_up_kcd_distance_report_table(nof_robots);
  /* clean up the report table for the movable objects */
  kcd_clean_up_kcd_distance_report_mos_table(nof_mos);
}




/* ******************************************************************* *
 * The following function are added to report the exact distance 
 * calculated by KCD. 
 * modifications done by Raeymaekers Pepijn (KINEO july 2001)
 * ******************************************************************* */

/* ******************************************************************* *
 * Function:  kcd_get_dist_mo_mo()
 *            returns for given movable object the distance estimate 
 *            stored in the kcd_distance_report_table_mos
 * ARGS IN : mo_id                   the movable object
 * ARGS OUT: d                       the distance estimate
 * PRE_COND: kcd_init_kcd_distance_report_table_mos() called before
 * ******************************************************************* */
void kcd_get_dist_mo_mo(int mo_id,double *d)
{ 
  *d = kcd_distance_report_table_mos[mo_id].min_distance;
}
/* ******************************************************************* *
 * Function:  kcd_set_dist_mo_mo()
 *            sets for given movable object the distance estimate 
 *            stored in the kcd_distance_report_table_mos
 * ARGS IN : mo_id                   the movable object
 * ARGS OUT: d                       the distance estimate
 * PRE_COND: kcd_init_kcd_distance_report_table_mos() called before
 * ******************************************************************* */
void kcd_set_dist_mo_mo(int mo_id,double d)
{ 
  kcd_distance_report_table_mos[mo_id].min_distance = d;
}

/* ******************************************************************* *
 * Function:  kcd_init_kcd_distance_report_table_mos() 
 *            initializes the distance table 
 *            for each movable object
 *            stored in the kcd_distance_report_table_mos
 * ARGS IN :  nof_mos       the number of movable objects
 * ******************************************************************* */
void kcd_init_kcd_distance_report_table_mos(int nof_mos)
{ 
  kcd_distance_report_table_mos = NULL;
  kcd_distance_report_table_mos = MY_ALLOC(kcd_closest_pair_struct_mos, nof_mos);

}
/* ******************************************************************* *
 * Function:  kcd_init_kcd_distance_report_table_mos_entry() 
 *            initializes the distance table entries
 *            for all movable objects
 *            stored in the kcd_distance_report_table_mos
 * ARGS IN :  nof_mos   the number of movable objects
 * ******************************************************************* */
void kcd_init_kcd_distance_report_table_mos_entry(int nof_mos)
{
  int i;
 
  for(i=0;i<nof_mos;i++)
    {
      kcd_distance_report_table_mos[i].min_distance = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].point_mo1[0] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo1[1] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo1[2] = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].point_mo2[0] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo2[1] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo2[2] = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].in_collision_with_mo_id = -1; 
    }
}
/* ******************************************************************* *
 * Function:  kcd_reset_kcd_distance_report_table_mos_entry() 
 *            reset the distance table entries
 *            for all movable objects
 *            stored in the kcd_distance_report_table_mos
 * ARGS IN :  nof_mos   the number of movable objects
 * ******************************************************************* */
void kcd_reset_kcd_distance_report_table_mos_entry(int nof_mos)
{
  int i;
  for(i=0;i<nof_mos;i++)
    {
      kcd_distance_report_table_mos[i].min_distance = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].point_mo1[0] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo1[1] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo1[2] = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].point_mo2[0] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo2[1] = P3D_HUGE;
      kcd_distance_report_table_mos[i].point_mo2[2] = P3D_HUGE;
      
      kcd_distance_report_table_mos[i].in_collision_with_mo_id = -1; 
    }
}

/* ******************************************************************* *
 * Init kcd_distance_report_table_mos
 * IN: /
 * USE: in kcd_end_scene() (kcd_api.c) 
 *      initialisation of the report table
 * ******************************************************************* */
void kcd_init_distance_report_table_mos()
{
  int nof_mos = kcd_get_nof_mos();
  kcd_init_kcd_distance_report_table_mos(nof_mos);
  kcd_init_kcd_distance_report_table_mos_entry(nof_mos);
}


/* ******************************************************************* *
 * Function:  kcd_clean_up_kcd_distance_report_mos_table()
 *            cleans the distance report table away for a nof_mos
 * ARGS IN :  nof_mos -> number of movable objects
 * ******************************************************************* */
void kcd_clean_up_kcd_distance_report_mos_table(int nof_mos)
{
  if(kcd_distance_report_table_mos)
    MY_FREE(kcd_distance_report_table_mos,kcd_closest_pair_struct_mos,nof_mos);
  kcd_distance_report_table_mos = NULL;
}
/* ******************************************************************* *
 * Function:  kcd_set_points_closest_pair_mos() 
 *            sets for given movable object the coordinates of the 
 *            closest pair of points and the corresponding closest
 *            movable object
 *         
 * ARGS IN : mo1 -> given movable object
 *           mo2 -> corresponding movable object
 *           point_mo1               coordinate of the closest point 
 *                                   of the given movable object
 *           point_mo2               coordinate of the closest point 
 *                                   of the corresponding  object
 * PRE_COND: kcd_init_kcd_distance_report_table_mos()
 * ******************************************************************* */
void kcd_set_points_closest_pair_mos(int mo1,int mo2, 
				     kcd_vector3 point_mo1, kcd_vector3 point_mo2)
{
  kcd_distance_report_table_mos[mo1].point_mo1[0] = point_mo1[0];
  kcd_distance_report_table_mos[mo1].point_mo1[1] = point_mo1[1];
  kcd_distance_report_table_mos[mo1].point_mo1[2] = point_mo1[2];

  kcd_distance_report_table_mos[mo1].point_mo2[0] = point_mo2[0];
  kcd_distance_report_table_mos[mo1].point_mo2[1] = point_mo2[1];
  kcd_distance_report_table_mos[mo1].point_mo2[2] = point_mo2[2];

  kcd_distance_report_table_mos[mo1].in_collision_with_mo_id = mo2;
}
/* ******************************************************************* *
 * Function:  kcd_get_points_closest_pair() 
 *            gets for given movable object the coordinates of the 
 *            closest pair of points and the corresponding movable
 *            object
 *         
 * ARGS IN : rob_i                   the group of movable objects
 *           body_id                 the movable object
 * ARGS OUT: point_mo               coordinate of the closest point 
 *                                   of the movable object
 *           point_so               coordinate of the closest point 
 *                                   of the static object
 *
 * The following information will be written in the report:
 * --------------------------------------------------------
 * For each body (mo) of each robot (group of mos) the position of the 
 * closest point will be given relative to the center of the body.
 ------> this is true for GJK_DEBUG 
 *
 * For each body (mo) of each robot (group of mos) the position of the 
 * closest point will be given relative to the environment (absolute position)
 ------> this is TRUE for KCD 
 *
 * The corresponding coordinates of the closest point will be given 
 * relative to the origin of the environment (absolute position).
 *
 * The points are not valid if the distance is zero !!
 * PRE_COND: kcd_init_kcd_distance_report_table_mos()  called before              
 * ******************************************************************* */
void kcd_get_points_closest_pair(int robot_id,int body_id, 
				 kcd_vector3 point_mo, kcd_vector3 point_so)
{
  point_mo[0] = kcd_distance_report_table[robot_id][body_id].point_mo[0];
  point_mo[1] = kcd_distance_report_table[robot_id][body_id].point_mo[1];
  point_mo[2] = kcd_distance_report_table[robot_id][body_id].point_mo[2];

  point_so[0] = kcd_distance_report_table[robot_id][body_id].point_so[0];
  point_so[1] = kcd_distance_report_table[robot_id][body_id].point_so[1];
  point_so[2] = kcd_distance_report_table[robot_id][body_id].point_so[2];
}


/* for a given mo we return 
   - the closest points to the nearest obstacle
*/
/* ******************************************************************* *
 * Function:  kcd_get_points_closest_pair_mos() 
 *            gets for given movable object in a given group of 
 *            movable objects the coordinates of the closest pair 
 *            of points
 *
 * ARGS IN : mo1 -> given movable object
 *   
 * ARGS OUT: 
 *           point_mo               coordinate of the closest point 
 *                                   of the given movable object
 *           point_so               coordinate of the closest point 
 *                                   of the corresponding  object 
 *
 * The following information will be written in the report:
 * --------------------------------------------------------
 * For each body (mo) the position of the 
 * closest point will be given relative to the environment 
 * (absolute position)
 *
 * The corresponding coordinates of the closest point will also be given 
 * relative to the origin of the environment (absolute position).
 *
 * The points are not valid if the distance is zero (collision exists)!!
 * 
 * PRE_COND: kcd_init_kcd_distance_report_table() called before
 *                
 * ******************************************************************* */
void kcd_get_points_closest_pair_mo_so(int mo1,
				       kcd_vector3 point_mo, kcd_vector3 point_so)
{
  point_mo[0] = kcd_dist_mo_report_table[mo1].point_mo[0];
  point_mo[1] = kcd_dist_mo_report_table[mo1].point_mo[1];
  point_mo[2] = kcd_dist_mo_report_table[mo1].point_mo[2];

  point_so[0] = kcd_dist_mo_report_table[mo1].point_so[0];
  point_so[1] = kcd_dist_mo_report_table[mo1].point_so[1];
  point_so[2] = kcd_dist_mo_report_table[mo1].point_so[2]; 
}
/* set the closest points */
void kcd_set_points_closest_pair_mo_so(int mo1,
				       kcd_vector3 point_mo, kcd_vector3 point_so)
{
  //  PrintInfo(("set closest points\n"));
  kcd_dist_mo_report_table[mo1].point_mo[0] = point_mo[0];
  kcd_dist_mo_report_table[mo1].point_mo[1] = point_mo[1];
  kcd_dist_mo_report_table[mo1].point_mo[2] = point_mo[2];
  
  kcd_dist_mo_report_table[mo1].point_so[0] = point_so[0];
  kcd_dist_mo_report_table[mo1].point_so[1] = point_so[1];
  kcd_dist_mo_report_table[mo1].point_so[2] = point_so[2]; 
}


/* for a given mo we return 
   - which other mo is the closest
   - and the closest points
*/
/* ******************************************************************* *
 * Function:  kcd_get_points_closest_pair_mos() 
 *            gets for given movable object in a given group of 
 *            movable objects the coordinates of the closest pair 
 *            of points
 *
 * ARGS IN : mo1 -> given movable object
 *   
 * ARGS OUT: mo2 -> corresponding movable object
 *           point_mo1               coordinate of the closest point 
 *                                   of the given movable object
 *           point_mo2               coordinate of the closest point 
 *                                   of the corresponding  object *

 * The following information will be written in the report:
 * --------------------------------------------------------
 * For each body (mo) the position of the 
 * closest point will be given relative to the environment 
 * (absolute position)
 ------> this is TRUE for KCD 
 *
 * The corresponding coordinates of the closest point will also be given 
 * relative to the origin of the environment (absolute position).
 *
 * The points are not valid if the distance is zero (collision exists)!!
 * 
 * PRE_COND: kcd_init_kcd_distance_report_table() called before
 *                
 * ******************************************************************* */
void kcd_get_points_closest_pair_mos(int mo1,int *mo2, 
				 kcd_vector3 point_mo1, kcd_vector3 point_mo2)
{
  point_mo1[0] = kcd_distance_report_table_mos[mo1].point_mo1[0];
  point_mo1[1] = kcd_distance_report_table_mos[mo1].point_mo1[1];
  point_mo1[2] = kcd_distance_report_table_mos[mo1].point_mo1[2];

  point_mo2[0] = kcd_distance_report_table_mos[mo1].point_mo2[0];
  point_mo2[1] = kcd_distance_report_table_mos[mo1].point_mo2[1];
  point_mo2[2] = kcd_distance_report_table_mos[mo1].point_mo2[2];

  *mo2 =  kcd_distance_report_table_mos[mo1].in_collision_with_mo_id;  
}

/* ******************************************************************* *
 * Function:  kcd_get_points_closest_pair_other_mo(int mo1)
 *            returns for given movable object the corresponding 
 *            closest movable object
 * ARGS IN : mo_id                   the movable object
 * ARGS OUT: the other movable object
 * ******************************************************************* */

int kcd_get_points_closest_pair_other_mo(int mo1)
{
  return (kcd_distance_report_table_mos[mo1].in_collision_with_mo_id); 
}

/* ******************************************************************* *
 * Function: kcd_reset_dist_mo_report_table(int nof_mos)
 *           reset the report tables for a the movable objects
 * ARGS IN : nof_mos -> number of movable objects
 * ******************************************************************* */
void kcd_reset_dist_mo_report_table(int nof_mos)
{
  int i; 
  for(i=0; i<nof_mos ; i++)
    {
      /* this table is used differently depending on which with_report
	 parameter is defined */ 
      kcd_dist_mo_report_table[i].min_distance = P3D_HUGE;
      /* init of closest points */
      //PrintInfo(("init closest points \n"));
      kcd_dist_mo_report_table[i].point_mo[0] = P3D_HUGE;
      kcd_dist_mo_report_table[i].point_mo[1] = P3D_HUGE;
      kcd_dist_mo_report_table[i].point_mo[2] = P3D_HUGE;
      
      kcd_dist_mo_report_table[i].point_mo[0] = P3D_HUGE;
      kcd_dist_mo_report_table[i].point_mo[1] = P3D_HUGE;
      kcd_dist_mo_report_table[i].point_mo[2] = P3D_HUGE;
    }
  kcd_reset_kcd_distance_report_table_mos_entry(nof_mos);
}
/* ******************************************************************* *
\fn void kcd_reset_dist_grp_so_report_table(int group_id)
\brief resets for a given group of movable objects the report 
concerning the closest points and distances with the static objects
 * ******************************************************************* */
void kcd_reset_dist_grp_so_report_table(int group_id)
{
  int i;   
  int nof_bodies = kcd_get_nof_grp_elts(group_id);
  for(i=0;i<nof_bodies;i++)
    {
    kcd_distance_report_table[group_id][i].min_distance = P3D_HUGE;

    kcd_distance_report_table[group_id][i].point_mo[0] = P3D_HUGE;
    kcd_distance_report_table[group_id][i].point_mo[1] = P3D_HUGE;
    kcd_distance_report_table[group_id][i].point_mo[2] = P3D_HUGE;
    
    kcd_distance_report_table[group_id][i].point_so[0] = P3D_HUGE;
    kcd_distance_report_table[group_id][i].point_so[1] = P3D_HUGE;
    kcd_distance_report_table[group_id][i].point_so[2] = P3D_HUGE;
    }
}
void kcd_reset_distance_report_table_grp_so()
{
  int nof_robots; 
  int rit;

  nof_robots = kcd_get_nof_grps();

  for(rit=0;rit<nof_robots;rit++)
    {
      kcd_reset_dist_grp_so_report_table(rit);
    }
}

