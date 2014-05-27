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
#include "Collision-pkg.h"

/* given the rank in which the object was given to KCD, get KCD-id,
   p3d_obj->o_id, and whether or not it is a movable object: */
typedef struct p3d_to_kcd
{
  int p3d_o_id;
  int kcd_obj_id;
  int is_movable;
}p3d_to_kcd, *p3d_to_kcd_p;

typedef int *int_p;

typedef struct p3d_kcd_env
{
  int             tot_nof_objs;
  int             p3d_env_id;
  int             kcd_scene_id;
  p3d_to_kcd_p    p3d_to_kcd_array;
  pp3d_obj      *o_id_to_obj_ptr;  
  int           *o_id_to_input_i;  
}p3d_kcd_env, *p3d_kcd_env_p;

/* stores global variables related to move3d-kcd interface for each environment */
static p3d_kcd_env_p p3d_kcd_env_arr = NULL;
static int nof_p3d_kcd_envs = 0;

/* given the rank in which the object was given to KCD, get KCD-id,
   p3d_obj->o_id, and whether or not it is a movable object: */
p3d_to_kcd_p p3d_to_kcd_array = NULL;
/* given the o_id of an object, get the pointer to the object */
pp3d_obj *o_id_to_obj_ptr = NULL; 
/* given the o_id of a p3d_obj, get rank in which the object was given to KCD: */
int *o_id_to_input_i = NULL; 

static int stats_graphic_prim = 0;
static int stats_robot_coll_prims = 0;


void p3d_kcd_set_object_moved(int o_id,int is_moved)
{
  /* we store that the object with unique move3d identifier o_id changed
     place recently (i.e. after the latest call to KCD collision test functions */
  int this_is_obj_i,this_is_kcd_id;

  this_is_obj_i = o_id_to_input_i[o_id];
  this_is_kcd_id = p3d_to_kcd_array[this_is_obj_i].kcd_obj_id;
  kcd_mo_moved(this_is_kcd_id,is_moved);
}

void add_p3d_kcd_env(int tot_nof_objs, int p3d_env_id, int kcd_scene_id)
{
/*   MY_ALLOC_INFO("p3d_kcd.c: before (RE)ALLOC"); */
  if(nof_p3d_kcd_envs == 0)
    {
/*       PrintInfo(("MY_ALLOC\n")); */
      p3d_kcd_env_arr = MY_ALLOC(p3d_kcd_env,1);
    }
  else
    {
/*       PrintInfo(("MY_REALLOC\n")); */
      p3d_kcd_env_arr = MY_REALLOC(p3d_kcd_env_arr,p3d_kcd_env,nof_p3d_kcd_envs,nof_p3d_kcd_envs+1);
    }
/*   MY_ALLOC_INFO("p3d_kcd.c: after (RE)ALLOC"); */
  p3d_kcd_env_arr[nof_p3d_kcd_envs].p3d_env_id = p3d_env_id;
  p3d_kcd_env_arr[nof_p3d_kcd_envs].kcd_scene_id = kcd_scene_id;
  p3d_kcd_env_arr[nof_p3d_kcd_envs].tot_nof_objs = tot_nof_objs;
  p3d_kcd_env_arr[nof_p3d_kcd_envs].p3d_to_kcd_array = p3d_to_kcd_array;
  p3d_kcd_env_arr[nof_p3d_kcd_envs].o_id_to_obj_ptr = o_id_to_obj_ptr;
  p3d_kcd_env_arr[nof_p3d_kcd_envs].o_id_to_input_i = o_id_to_input_i;

  nof_p3d_kcd_envs++;
}

void deconnect_p3d_kcd_global_vars()
{
  p3d_to_kcd_array = NULL;
  o_id_to_obj_ptr = NULL;
  o_id_to_input_i = NULL;
}

void connect_p3d_kcd_global_vars(int kcd_scene_id)
{
/*   PrintInfo(("connect_p3d_kcd_global_vars(%i)\n",kcd_scene_id)); */
  p3d_to_kcd_array = p3d_kcd_env_arr[kcd_scene_id].p3d_to_kcd_array;
  o_id_to_obj_ptr  = p3d_kcd_env_arr[kcd_scene_id].o_id_to_obj_ptr;
  o_id_to_input_i  = p3d_kcd_env_arr[kcd_scene_id].o_id_to_input_i;
}

void clean_up_p3d_kcd_env_arr()
{
  int i,nof_objs;

  for(i=0;i<nof_p3d_kcd_envs;i++)
    {
      nof_objs = p3d_kcd_env_arr[i].tot_nof_objs;

      MY_FREE(p3d_kcd_env_arr[i].p3d_to_kcd_array,p3d_to_kcd,nof_objs);
      MY_FREE(p3d_kcd_env_arr[i].o_id_to_obj_ptr,pp3d_obj,nof_objs);
      MY_FREE(p3d_kcd_env_arr[i].o_id_to_input_i,int,nof_objs);
    }

  MY_FREE(p3d_kcd_env_arr,p3d_kcd_env,nof_p3d_kcd_envs);
  p3d_kcd_env_arr = NULL;

  nof_p3d_kcd_envs = 0;
}


pp3d_obj get_obj_ptr_from_o_id(int mo_o_id)
{
  /* PrintInfo(("get_obj_ptr_from_o_id(%i)\n",mo_o_id)); */
  return (o_id_to_obj_ptr[mo_o_id]);  
}

void p3d_kcd_get_pairObjInCollision(p3d_obj **o1Pt,p3d_obj **o2Pt)
{
  int o1,o2;

  kcd_get_pairObjInCollision(&o1,&o2);
  *o1Pt = get_obj_ptr_from_o_id(o1);
  *o2Pt = get_obj_ptr_from_o_id(o2);
}

void p3d_get_mo_pos(int ext_o_id, poly_matrix4 **pos)
{
  p3d_obj *mov_obj;

  mov_obj = get_obj_ptr_from_o_id(ext_o_id);
  /* PrintInfo(("-> %s\n",mov_obj->name)); */
  if(mov_obj->jnt)
    *pos = &(mov_obj->jnt->abs_pos);
}

/* get AABB around the kcd_ext_o-th movable object given by user */
/* returns FALSE if user cannot give this information */
int p3d_get_aabb_on_mo(int kcd_ext_o, 
			double *x1,double *x2,double *y1,
			double *y2,double *z1,double *z2)
{
  int o_id;
  pp3d_obj obj_ptr;

  o_id=get_p3d_id_from_input_index(kcd_ext_o);
  obj_ptr = get_obj_ptr_from_o_id(o_id);
  p3d_get_BB_obj(obj_ptr,x1,x2,y1,y2,z1,z2);
/*   PrintInfo(("Move3D AABB of mov.obj.: x1=%f,x2=%f,y1=%f,y2=%f,z1=%f,z2=%f\n", */
/* 	 *x1,*x2,*y1,*y2,*z1,*z2); */

  /* return FALSE; */ /* just to test 17 01 2001 */
  return TRUE; /* Move3D knows how to compute */
}

/*!
  Get the kcd id of a p3d_obj, and determine wether it is a movable object.

  \param o: an object.
  \param movable: output parameter, is \a o a movable object ?

  \return The kcd id of the object.
 */
int get_kcd_id_from_object(p3d_obj* o, bool& movable)
{
  movable = p3d_to_kcd_array[o_id_to_input_i[o->o_id_in_env]].is_movable;
  return(p3d_to_kcd_array[o_id_to_input_i[o->o_id_in_env]].kcd_obj_id);
}

int get_kcd_id_from_input_index(int input_index, int *is_movable)
{
  *is_movable = p3d_to_kcd_array[input_index].is_movable;
  return (p3d_to_kcd_array[input_index].kcd_obj_id);
}

int get_p3d_id_from_input_index(int input_index)
{
  return (p3d_to_kcd_array[input_index].p3d_o_id);
}

void kcd_obb_construct(int nof_graphic_obst)
{
  int tot_nof_objs,gt,this_is_obj_i=-1;
  int is_robot = FALSE;
  int i,j,k,poly_so_far=0;
  p3d_poly *m3d_poly_it = NULL;
  int poly_obb_nr;
  p3d_rob *rob_it;
  p3d_obj *obj_it;
  int **arr_mo_id=NULL;
  int kcd_obj_id;
  /* int no_graphic_body = 0;  Modif Fabien */

  tot_nof_objs = XYZ_ENV->nof_objs;

/*   PrintInfo(("\n tot_nof_objs = %i \n",tot_nof_objs)); */
/*   MY_ALLOC_INFO("p3d_kcd.c: alloc voor"); */
  p3d_to_kcd_array = NULL;
  p3d_to_kcd_array = MY_ALLOC(p3d_to_kcd,tot_nof_objs);
  o_id_to_input_i = NULL;
  o_id_to_input_i = MY_ALLOC(int,tot_nof_objs);      
  o_id_to_obj_ptr = NULL;
  o_id_to_obj_ptr = MY_ALLOC(pp3d_obj,tot_nof_objs); 
/*   MY_ALLOC_INFO("p3d_kcd.c: alloc  na "); */

  /* walk through all polyhedrons (and solids) */
  /* static objects */
  for(i=0;i<XYZ_ENV->no;i++)
    {
      gt = XYZ_ENV->o[i]->GRAPHIC_TYPE;
      if( (gt != P3D_GRAPHIC_OBJECT) && (gt != P3D_ADDABLE_OBSTACLE) )
	{
	  obj_it = XYZ_ENV->o[i];
	  this_is_obj_i++;
	  o_id_to_input_i[obj_it->o_id_in_env] = this_is_obj_i; 
	  o_id_to_obj_ptr[obj_it->o_id_in_env] = obj_it;      

	  kcd_beg_obj(FALSE);  /* FALSE: it's a static object */

	  for(j=0;j<obj_it->np;j++)
	    {
	      m3d_poly_it = obj_it->pol[j];
	      if(m3d_poly_it->TYPE != P3D_GRAPHIC)
		{
		  if( p3d_filter_relevant_poly(m3d_poly_it) ) /* this line added: FILTER */
		    poly_obb_nr = kcd_add_prim(m3d_poly_it);

		  poly_so_far++;
		}
	      else
		{
		  stats_graphic_prim++;
		  /* this polyhedron is P3D_GRAPHIC and must be ignored 
		     by collision detector */
		}
	    }

	  kcd_obj_id = kcd_end_obj();
	  /* NEW: */
	  p3d_to_kcd_array[this_is_obj_i].p3d_o_id = obj_it->o_id_in_env;
	  /* WAS: */
	  /* (*p3d_to_kcd_array)[this_is_obj_i].p3d_o_id = obj_it->o_id; */
	  p3d_to_kcd_array[this_is_obj_i].kcd_obj_id = kcd_obj_id;
	  p3d_to_kcd_array[this_is_obj_i].is_movable = FALSE;
	}
    }
  /* robots */
  is_robot = TRUE;
  arr_mo_id = NULL;
  arr_mo_id = MY_ALLOC(int_p,XYZ_ENV->nr);
  /*  graphic = MY_ALLOC(int_p,XYZ_ENV->nr);  Modif Fabien */

  for(k=0;k<XYZ_ENV->nr;k++)
    {
      rob_it = XYZ_ENV->robot[k];
      /* graphic[k] = NULL;  
      graphic[k] = MY_ALLOC(int,rob_it->no);   Modif Fabien */
      arr_mo_id[k] = NULL;
      arr_mo_id[k] = MY_ALLOC(int,rob_it->no);
      /* links */
      for(i=0;i<rob_it->no;i++)
	{
	  obj_it = rob_it->o[i];
	  this_is_obj_i++;
	  /* NEW: */
	  o_id_to_input_i[obj_it->o_id_in_env] = this_is_obj_i; 
	  o_id_to_obj_ptr[obj_it->o_id_in_env] = obj_it;        
	  /* WAS: */
	  /* (*o_id_to_input_i)[obj_it->o_id] = this_is_obj_i; */  /* 24/10/01 */
	  /* (*o_id_to_obj_ptr)[obj_it->o_id] = obj_it; */         /* 24/10/01 */

	  kcd_beg_obj(TRUE);  /* TRUE: it's a movable object */

	  /* modif pepijn aout 2001 */
	  /* graphic[k][i] = 0;
	  no_graphic_body = 0;  Modif Fabien */
	  for(j=0;j<obj_it->np;j++)
	    {
	      m3d_poly_it = obj_it->pol[j];
	      if(m3d_poly_it->TYPE != P3D_GRAPHIC)
		{
		  stats_robot_coll_prims++;

		  poly_obb_nr = kcd_add_prim(m3d_poly_it);
		  /* no_graphic_body++;   Modif Fabien */
		  poly_so_far++;
		}
	      else
		{
		  /* this polyhedron is P3D_GRAPHIC and must be 
		     ignored by collision detector */
		  stats_graphic_prim++;
		}
	    }
	  /* Modif Fabien
	  if(no_graphic_body == 0)
	    {	    
	      graphic[k][i] = TRUE;
	    } */
	  kcd_obj_id = kcd_end_obj();
	  /* NEW: */
	  p3d_to_kcd_array[this_is_obj_i].p3d_o_id = obj_it->o_id_in_env;
	  /* WAS: */
	  /* (*p3d_to_kcd_array)[this_is_obj_i].p3d_o_id = obj_it->o_id; */
	  p3d_to_kcd_array[this_is_obj_i].kcd_obj_id = kcd_obj_id;
	  p3d_to_kcd_array[this_is_obj_i].is_movable = TRUE;
	  /* for below: group definition: */
	  arr_mo_id[k][i] = kcd_obj_id;
	}
    }
  /* define the robots for KCD: make groups of movable objects */
  for(k=0;k<XYZ_ENV->nr;k++)
    {
      rob_it = XYZ_ENV->robot[k];      
      kcd_def_mo_grp(arr_mo_id[k],rob_it->no);
      MY_FREE(arr_mo_id[k],int,rob_it->no);
      arr_mo_id[k] = NULL;
    }
  kcd_def_mo_no_grp(); /* Movable object without robot */
  MY_FREE(arr_mo_id,int_p,XYZ_ENV->nr);
  arr_mo_id = NULL;

  /* communicate complexity of the scene */
  PrintInfo(("KCD: nof solids = %i, nof polyhs = %i, nof graphic prims = %i,\n robot primitive to test = %i\n",
	 get_stats_nof_solids(),get_stats_nof_polyhs(),stats_graphic_prim,stats_robot_coll_prims));
}

int p3d_kcd_test_self_collision(int robot_number, int with_report)
{
  double d;
  int i;
  kcd_set_report_to_zero();
  return kcd_robot_collides_itself(robot_number, with_report, &d, &i);
}


int p3d_kcd_test_robot_statics(p3d_rob *robotPt, int with_report)
{
  double kcd_distance_estimate = 0.0;
  int kcd_nearest_obst = 0;
  kcd_set_report_to_zero();
  return ( kcd_robot_collides(robotPt->num, with_report,&kcd_distance_estimate, &kcd_nearest_obst) );	  
}


int p3d_kcd_test_robot_other(p3d_rob *robotPt1, p3d_rob *robotPt2, int with_report)
{
  double dist;
  int obst;
  kcd_set_report_to_zero();
  return ( kcd_robot_vs_robot(robotPt1->num, robotPt2->num, with_report,&dist,&obst));
}



int p3d_kcd_collision_test()
{
  return kcd_collision_exists(NO_REPORT,NULL);
}

int p3d_kcd_collision_test_all()
{
  return kcd_collision_exists(JUST_BOOL,NULL);
}



int p3d_kcd_test_everything(int kcd_with_report)
{
  int result;
  double min_dist;
  result = kcd_collision_exists( kcd_with_report, &min_dist);
  return result;
}

int p3d_kcd_collision_test_and_distance_estimate(double *min_distance_estimate)
{
  int result;

  *min_distance_estimate = P3D_HUGE;
  result = kcd_collision_exists(DISTANCE_ESTIMATE,min_distance_estimate);
  /* PrintInfo(("p3d_kcd_collision_test_and_distance_estimate: dist = %f\n",*min_distance_estimate)); */
  return result;
}


void p3d_kcd_set_pos(p3d_poly *p, p3d_matrix4 mat)
{
  /* doesn't do anything */
}

void p3d_start_kcd(void)
{
  int p3d_env_id,kcd_scene_id,scene_stored = FALSE,good_val = 0;
  int i,j,k,nof_polyh=0,nof_obst,nof_graphic_obst,gt,nof_bodies,nof_robs;

  /* basic_alloc_debugon(); */
  if(!get_collision_by_object())
    {
      set_collision_by_object(TRUE);
      PrintInfo(("KCD WARNING: set COLLISION_BY_OBJECT TRUE\n"));
    }

  deconnect_p3d_kcd_global_vars();
  
  p3d_env_id = XYZ_ENV->num;

  for(i=0;(i<nof_p3d_kcd_envs)&&(!scene_stored);i++)
    {
      scene_stored = (p3d_env_id == p3d_kcd_env_arr[i].p3d_env_id);
      if(scene_stored)
	good_val = i;
      /* PrintInfo(("p3d_start_kcd(): loop: i=%i\n",i)); */
    }
  
  if(scene_stored)
    {
      /* PrintInfo(("p3d_start_kcd(): out of loop: i=%i\n",good_val)); */
      kcd_scene_id = p3d_kcd_env_arr[good_val].kcd_scene_id;
      connect_p3d_kcd_global_vars(kcd_scene_id);
      kcd_remember_scene(kcd_scene_id);
    }
  else
    {
      /* begin addition: FILTER */
      if(p3d_filter_needs_init())
	p3d_filter_init_filter();
      /*  end  addition: FILTER */

      nof_obst = XYZ_ENV->no;
      nof_graphic_obst = 0;
      for(i=0;i<nof_obst;i++)
	{
	  gt = XYZ_ENV->o[i]->GRAPHIC_TYPE;
	  if( (gt != P3D_GRAPHIC_OBJECT) && (gt != P3D_ADDABLE_OBSTACLE) )
	    {
	      for(j=0;j<XYZ_ENV->o[i]->np;j++)
		if(XYZ_ENV->o[i]->pol[j]->TYPE != P3D_GRAPHIC)
		  nof_polyh++; 
	    }
	  else
	    {
	      nof_graphic_obst++;
	    }
	}
      nof_robs = XYZ_ENV->nr;
      for(i=0;i<nof_robs;i++)
	{
	  nof_bodies = XYZ_ENV->robot[i]->no;
	  for(j=0;j<nof_bodies;j++)
	    {
	      for(k=0;k<XYZ_ENV->robot[i]->o[j]->np;k++)
		if(XYZ_ENV->robot[i]->o[j]->pol[k]->TYPE != P3D_GRAPHIC)
		  nof_polyh++;
	    }
	}
      nof_bodies = p3d_get_desc_number(P3D_BODIES);
      
      kcd_beg_scene(nof_polyh,nof_obst-nof_graphic_obst,nof_robs,nof_bodies);
      
      /* walk through all polyhedrons (and solids) */
      /* put a OBB around this polyhedron (or solid) */
      kcd_obb_construct(nof_graphic_obst);
      
      kcd_scene_id = kcd_end_scene();
      /* PrintInfo(("p3d_start_kcd(): kcd_scene_id = %i\n",kcd_scene_id)); */
      /* kcd_end_scene does: */
	 /* 1. make access to kcd_bb pile */
      /*      make_hash_table_on_bbs();  */
      /* 2. put OBB-tree on top of OBB-roots in each of the robot bodies */
      /*      kcd_obb_construct_on_links(); */
      /* 3. put a AABB around this polyhedron (or solid) */
      /*      kcd_aabb_construct();  */
      /* 4. n_of_bbs = kcd_get_number_of_bbs(); */
      /*      kcd_gjk_support_initialize_interesting_seed(n_of_bbs); */
      /* 5. initialize table to store distances between bodies and their nearest static obstacle */
      /* 5. put a box around devices, freights, movable, and */
      /*    deformable objects and initialize the overlapping AABBs */
      /* 6.   kcd_init_movable_stuff(); */
      /* 7.   kcd_init_distance_report_table(); */
       

      /* initialize pairs that must be tested */
//       p3d_col_activate_robots();

      /* Modification Pepijn august 2001 elimination BUG P3D_GRAPHIC*/
      /* kcd_disable_graphic_body_testing(); checked in p3d_col_activate_robots */
      
      /* store some global variables used in this file, in the array of environment data */
      add_p3d_kcd_env(XYZ_ENV->nof_objs,p3d_env_id,kcd_scene_id);

      /* begin test */
/*       deconnect_p3d_kcd_global_vars(); */
/*       connect_p3d_kcd_global_vars(kcd_scene_id); */
      /*  end  test */
    }
  /* kcd_is_initialized = TRUE; */
}


/* void p3d_col_stop(void) */
void p3d_kcd_stop()
{
  int i; 
  
  if(p3d_col_get_mode() == p3d_col_mode_kcd) {
    /* p3d_filter_cleanup(); */ /* this line added: FILTER */
    
    /* kcd_clean_up_distance_report_table(); */
    for(i=0;i<XYZ_ENV->nr;i++)
      {
	kcd_delete_movable_stuff_robot(i);
      }
    
    /*   MY_ALLOC_INFO("p3d_kcd.c:before p3d_kcd_env_arr()"); */
    clean_up_p3d_kcd_env_arr();
    /* free all environment information used in this file */
    deconnect_p3d_kcd_global_vars();
 /* free kcd internal data structures */
    /*   MY_ALLOC_INFO("p3d_kcd.c:after p3d_kcd_env_arr()"); */
    kcd_clean_up();
    /*   MY_ALLOC_INFO("p3d_kcd.c:kcd_clean_up() done"); */
    
    /* basic_alloc_debugoff(); */
    
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision and the distance computation
 *        between two movable objects.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_pair_of_objects(kcd_col_handle * handlePt, 
			     p3d_obj *body1, p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env];
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env];

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(!p3d_col_object_is_pure_graphic(body1) && 
     !p3d_col_object_is_pure_graphic(body2) && is_mo1 && is_mo2)
    { kcd_act_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision and the distance computation
 *        between two movable objects.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_pair_of_objects(kcd_col_handle * handlePt,
			       p3d_obj *body1,p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env]; 
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env]; 

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(is_mo1 && is_mo2)
    { kcd_deact_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision between two movable objects.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note Do not modify the distance conputation selection.
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_collision_pair_of_objects(kcd_col_handle * handlePt, 
				       p3d_obj *body1, p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env];
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env];

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(!p3d_col_object_is_pure_graphic(body1) && 
     !p3d_col_object_is_pure_graphic(body2) && is_mo1 && is_mo2)
    { kcd_act_col_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision between two movable objects.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note Do not modify the distance conputation selection.
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_collision_pair_of_objects(kcd_col_handle * handlePt,
					 p3d_obj *body1,p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env]; 
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env]; 

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(is_mo1 && is_mo2)
    { kcd_deact_col_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the distance computation between two movable objects.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note Do not modify the test of collision selection.
 * \note If the groups of those two movable objects have no test between them,
 *       then it activate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_distance_pair_of_objects(kcd_col_handle * handlePt, 
				      p3d_obj *body1, p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env];
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env];

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(!p3d_col_object_is_pure_graphic(body1) && 
     !p3d_col_object_is_pure_graphic(body2) && is_mo1 && is_mo2)
    { kcd_act_dist_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the distance computation between two movable objects.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \note Do not modify the test of collision selection.
 * \note If the groups of those two movable objects have no test between them,
 *       then it deactivate the collision between those two groups.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_distance_pair_of_objects(kcd_col_handle * handlePt,
					 p3d_obj *body1,p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env]; 
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env]; 

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(is_mo1 && is_mo2)
    { kcd_deact_dist_mo_pair(handlePt, kcd_o_id1, kcd_o_id2); }
  else {
    PrintWarning(("KCD: Warning: one of the collision pair objects is not movable\n"));
    PrintWarning(("              pair not added\n"));
  }
}


/* B Kineo Carl 27.02.2002 */
/* (re-)activate obstacle, obstacles are active by default */
void p3d_kcd_activate_obstacle(p3d_obj *obstaclePt)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[obstaclePt->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);

  if(!is_mo)
    {
      kcd_act_obst(kcd_o_id);
    }
  else
    {
      PrintWarning(("KCD: Warning: the object is not static\n"));
    }
}

/* deactivate obstacle, obstacles are active by default */
void p3d_kcd_deactivate_obstacle(p3d_obj *obstaclePt)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[obstaclePt->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);

  if(!is_mo)
    {
      kcd_deact_obst(kcd_o_id);
    }
  else
    {
      PrintWarning(("KCD: Warning: the object is not static\n"));
    }
}
/* E Kineo Carl 27.02.2002 */


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision and the distance computation between
 *        a movable object and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_object_to_env(kcd_col_handle * handlePt, p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(!p3d_col_object_is_pure_graphic(body) && is_mo)
    { kcd_act_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not activate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision and the distance computation between
 *        a movable object and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_object_to_env(kcd_col_handle * handlePt, 
			     p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(is_mo)
    { kcd_deact_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not deactivate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision between
 *        a movable object and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note Do not modify the distance conputation selection.
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_collision_object_to_env(kcd_col_handle * handlePt, p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(!p3d_col_object_is_pure_graphic(body) && is_mo)
    { kcd_act_col_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not activate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision between
 *        a movable object and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note Do not modify the distance conputation selection.
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_collision_object_to_env(kcd_col_handle * handlePt, 
				       p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(is_mo)
    { kcd_deact_col_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not deactivate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the distance computation between
 *        a movable object and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note Do not modify the test of collision selection.
 * \note If the group of this movable object has no test with the environment,
 *       then it activate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_add_distance_object_to_env(kcd_col_handle * handlePt, p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(!p3d_col_object_is_pure_graphic(body) && is_mo)
    { kcd_act_dist_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not activate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the distance computation between
 *        a movable object and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \note Do not modify the test of collision selection.
 * \note If the group of this movable object has no test with the environment,
 *       then it deactivate the collision for the group.
 * \note In other case it doesn't change the activation between groups.
 */
void kcd_deact_distance_object_to_env(kcd_col_handle * handlePt, 
				      p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(is_mo)
    { kcd_deact_dist_mo_env(handlePt, kcd_o_id); }
  else {
    PrintWarning(("KCD: Warning: the collision pair object is not movable\n"));
    PrintWarning(("              pair with the environment not deactivate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision between two robots.
 *
 * \param  handlePt: The kcd collision handle.
 * \param  rob1:     The first robot.
 * \param  rob2:     The second robot.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by 
 *       kcd_add_collision_pair_of_objects() or
 *       kcd_deact_collision_pair_of_objects().
 *
 * \note If (\a rob1 == \a rob2) then this active the autocollision.
 */
void kcd_add_collision_robot_pair(kcd_col_handle * handlePt,
				  p3d_rob * rob1, p3d_rob * rob2)
{
  if((rob1!=NULL) && (rob2!=NULL))
    { kcd_act_grp_pair(handlePt, rob1->num, rob2->num); }
  else {
    PrintWarning(("KCD: Warning: the robts are not valid\n"));
    PrintWarning(("              pair not activate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision between two robots.
 *
 * \param  handlePt:  The kcd collision handle.
 * \param  rob1:     The first robot.
 * \param  rob2:     The second robot.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by
 *       kcd_add_collision_pair_of_objects() or
 *       kcd_deact_collision_pair_of_objects().
 *       A call to kcd_add_collision_robot_pair() restore all those links.
 * \note if (\a rob1 == \a rob2) then this deactive the autocollision.
 */
void kcd_deact_collision_robot_pair(kcd_col_handle * handlePt,
				    p3d_rob * rob1, p3d_rob * rob2)
{
  if((rob1!=NULL) && (rob2!=NULL))
    { kcd_deact_grp_pair(handlePt, rob1->num, rob2->num); }
  else {
    PrintWarning(("KCD: Warning: the robots are not valid\n"));
    PrintWarning(("              pair not deactivate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Activate the test of collision between a robot and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param  rob:      The robot.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by 
 *       kcd_add_collision_object_to_env() or
 *       kcd_deact_collision_object_to_env().
 */
void kcd_add_collision_robot_to_env(kcd_col_handle * handlePt, p3d_rob * rob)
{
  if(rob!=NULL)
    { kcd_act_grp_env(handlePt, rob->num); }
  else {
    PrintWarning(("KCD: Warning: the robt is not valid\n"));
    PrintWarning(("              pair not activate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate the test of collision between a robot and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 * \param  rob:      The robot.
 *
 * \note This function doesn't change the links between movable object.
 *       This links could be modified only by 
 *       kcd_add_collision_object_to_env() or
 *       kcd_deact_collision_object_to_env().
 *       A call to kcd_add_collision_robot_to_env() restore all those links.
 */
void kcd_deact_collision_robot_to_env(kcd_col_handle * handlePt,
				      p3d_rob * rob)
{
  if (rob!=NULL)
    { kcd_deact_grp_env(handlePt, rob->num); }
  else {
    PrintWarning(("KCD: Warning: the robot is not valid\n"));
    PrintWarning(("              pair not deactivate\n"));
  }
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Deactivate all the test of collision.
 *
 * \param handlePt:   The kcd collision handle.
 */
void kcd_deactivate_all(kcd_col_handle * handlePt)
{
  kcd_deactivate_all_mo(handlePt);
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a test of collision between two objects.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between their groups are
 *          deactivate and so even if no collision test is done.
 */
int kcd_is_collision_pair_of_objects(kcd_col_handle * handlePt, 
				     p3d_obj *body1, p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env];
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env];

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(is_mo1 && is_mo2)
    { return kcd_col_mo_pair_is_act(handlePt, kcd_o_id1, kcd_o_id2); }

  return FALSE;
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a distance computed between two objects.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body1:     The first movable object (body).
 * \param body2:     The second movable object (body).
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between their groups are
 *          deactivate and so even if no collision test is done.
 */
int kcd_is_distance_pair_of_objects(kcd_col_handle * handlePt, 
				     p3d_obj *body1, p3d_obj *body2)
{
  int kcd_o_id1,kcd_o_id2;
  int is_mo1, is_mo2;
  int kcd_input_index1,kcd_input_index2;

  kcd_input_index1 = o_id_to_input_i[body1->o_id_in_env];
  kcd_input_index2 = o_id_to_input_i[body2->o_id_in_env];

  kcd_o_id1 = get_kcd_id_from_input_index(kcd_input_index1,&is_mo1);
  kcd_o_id2 = get_kcd_id_from_input_index(kcd_input_index2,&is_mo2);
  
  if(is_mo1 && is_mo2)
    { return kcd_dist_mo_pair_is_act(handlePt, kcd_o_id1, kcd_o_id2); }

  return FALSE;
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a collision test between an object
 *        and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between its group and the
 *          environment is deactivate and so even if no collision test is done.
 */
int kcd_is_collision_object_to_env(kcd_col_handle * handlePt, p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(is_mo)
    { return kcd_col_mo_env_is_act(handlePt, kcd_o_id); }

  return FALSE;
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a distance computed between an object
 *        and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param body:      The movable object (body).
 *
 * \return TRUE if given pair of movable objects exists and is activated,
 *         FALSE otherwise 
 *
 * \remarks Returns TRUE even if the collision between its group and the
 *          environment is deactivate and so even if no collision test is done.
 */
int kcd_is_distance_object_to_env(kcd_col_handle * handlePt, p3d_obj *body)
{
  int kcd_o_id;
  int is_mo;
  int kcd_input_index;

  kcd_input_index = o_id_to_input_i[body->o_id_in_env];
  kcd_o_id = get_kcd_id_from_input_index(kcd_input_index,&is_mo);
  
  if(is_mo)
    { return kcd_dist_mo_env_is_act(handlePt, kcd_o_id); }

  return FALSE;
}


/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a test of collision between two robots.
 *
 * \param handlePt:  The kcd collision handle.
 * \param rob1:      The first robot.
 * \param rob2:      The second robot.
 *
 * \return TRUE if given pair of robots exists and is activated,
 *         FALSE otherwise 
 */
int kcd_is_collision_robot_pair(kcd_col_handle * handlePt, 
				p3d_rob *rob1, p3d_rob *rob2)
{  
  if((rob1 != NULL) && (rob2 != NULL))
    { return kcd_grp_pair_is_act(handlePt, rob1->num, rob2->num); }
  return FALSE;
}

/*--------------------------------------------------------------------------
 * KCD_API VERSION: */
/*!
 * \brief Returns if there is a test of collision between a robot
 *        and the environment.
 *
 * \param handlePt:  The kcd collision handle.
 * \param rob:       The robot.
 *
 * \return TRUE if given pair of robots exists and is activated,
 *         FALSE otherwise 
 */
int kcd_is_collision_robot_to_env(kcd_col_handle * handlePt, p3d_rob *rob)
{  
  if(rob != NULL)
    { return kcd_grp_env_is_act(handlePt, rob->num); }
  return FALSE;
}

/* kcd get corresponding mo_id used in kcd for the body */
int p3d_kcd_get_cor_mo(int o_id)
{ 
  int kcd_obj_id = p3d_to_kcd_array[o_id_to_input_i[o_id]].kcd_obj_id;  
  return (kcd_report_get_cor_mo(kcd_obj_id));
}

/****************************************************************************
 * Function returns an array of distances  (*distances) computed by the      
 * collision detector between the links of a given robot 
 *
 * In :  robot pointer      
 *       space for array of distances ->  [nof_bodies]
 *       space for matrix: points of movable objects ->  [3][nof_bodies]
 *       space for matrix:  points of static  objects ->  [3][nof_bodies] 
 * Out : distances[i]:       is distance for body i to closest other body
 *       points_mos1[i][3]
 *       points_mos2[i][3]: the corresponding closest point relative to 
 *                          the environment 
 *
 ****************************************************************************/

void p3d_kcd_closest_points_between_bodies(p3d_rob *robotPt,
					   p3d_vector3 *points_mos1,
					   p3d_vector3 *points_mos2, 
					   double *distances)
{
  int i, j, nof_bodies, mo1,mo2;
  double min;
  p3d_vector3 zpa,zpb;


  nof_bodies = robotPt->no;
  for(j=0;j<nof_bodies;j++)
    {
      /* NEW: */
      mo1 = p3d_kcd_get_cor_mo(robotPt->o[j]->o_id_in_env);
      /* WAS: */
      /* mo1 = p3d_kcd_get_cor_mo(robotPt->o[j]->o_id); */
      kcd_get_points_closest_pair_mos(mo1,&mo2,zpa,zpb);
      for (i = 0 ; i < 3 ; i++)
	{
	  points_mos1[j][i]  = zpa[i]; 
	  points_mos2[j][i]  = zpb[i]; 
	}     
      kcd_get_dist_mo_mo(mo1, &min);
      distances[j] = min;
    }
}

/****************************************************************************
 * Function returns an array of distances  (*distances) computed by the      
 * collision detector between the links of a given robot and the environment
 *
 * In :  robot pointer      
 *       space for array of distances ->  [nof_bodies]
 *       space for matrix: points of movable objects ->  [3][nof_bodies]
 *       space for matrix:  points of static  objects ->  [3][nof_bodies] 
 * Out : distances[i] is distance estimate between body i and closest so
 *       points_mos[i][3] coordinates of the point of the body (mo) that
 *                        is closest to the so  (relative to the origine of the
 *                        coordinate frame (=environment))
 *       points_sos[i][3] coordinates of the point of the object (so) that
 *                        is closest to the mo (relative to the origine of the
 *                        coordinate frame)
 *
 ****************************************************************************/
void p3d_kcd_closest_points_robot_environment(p3d_rob *robotPt, p3d_vector3 *points_mos, 
					     p3d_vector3 *points_sos, double *distances)
{

  int i, j, nof_bodies;
  double min;
  p3d_vector3 zpa,zpb;

  nof_bodies = robotPt->no;
  for(j=0;j<nof_bodies;j++)
    {
      kcd_get_points_closest_pair(robotPt->num,j,zpa,zpb);
      for (i = 0 ; i < 3 ; i++)
	{
	  points_mos[j][i]  = zpa[i]; 
	  points_sos[j][i]  = zpb[i]; 
	}     
      kcd_get_dist_grp_mo_so(robotPt->num,j,&min);
      distances[j] = min;
    }
}

/*!
\brief get the real closest points( test is made between closest points to moving objects
and closest points to static objects)
\param distances[i] is distance estimate between body i and closest so
\param points_mos[i][3] coordinates of the point of the body (mo) that
                        is closest to the so  (relative to the origine of the
                        coordinate frame (=environment))
\param points_sos[i][3] coordinates of the point of the object (so) that
                        is closest to the mo (relative to the origine of the
                        coordinate frame)
*/

void p3d_kcd_closest_points_of_both(p3d_rob *robotPt, p3d_vector3 *points_mos, 
				    p3d_vector3 *points_sos, double *distances)
{

  int i, j, nof_bodies,mo1,mo2;
  double d_body,d_obst;
  p3d_vector3 zpa,zpb;

  nof_bodies = robotPt->no;
  for(j=0;j<nof_bodies;j++)
    {   
      /* NEW: */
      mo1 = p3d_kcd_get_cor_mo(robotPt->o[j]->o_id_in_env);
      /* WAS: */
      /* mo1 = p3d_kcd_get_cor_mo(robotPt->o[j]->o_id); */
      kcd_get_dist_mo_mo(mo1, &d_body);
      kcd_get_dist_grp_mo_so(robotPt->num,j,&d_obst);
      if(d_body < d_obst)
	{     
	  distances[j] = d_body;
	  kcd_get_points_closest_pair_mos(mo1,&mo2,zpa,zpb);
	}
      else
	{
	  distances[j] = d_obst;	 
	  kcd_get_points_closest_pair(robotPt->num,j,zpa,zpb);
	}
      for (i = 0 ; i < 3 ; i++)
	{
	  points_mos[j][i]  = zpa[i]; 
	  points_sos[j][i]  = zpb[i]; 
	}     
     
    }
}


/* pre-condition: test between bodies must have been done *
 *   depending on how KCD is used we get different distances                    *
 *   MORE INFORMATION: see p3d.kcd.c -> different ways to use KCD
 *   (modif Pepijn Raeymaekers KINEO (july 2001))
 *   returns the latest set of distances computed between the links of the      *
 *   given robot and other moving objects                                 */
void kcd_get_dist_for_bodies(int robot_id, double *distances)
{
  int body_id,jnt_id=0,nof_joints = XYZ_ENV->robot[robot_id]->njoints+1;    
  p3d_obj *bodyPt;
  double d;
  int mo1;

  for(jnt_id=0;jnt_id<nof_joints;jnt_id++)
    {
      bodyPt = XYZ_ENV->robot[robot_id]->joints[jnt_id]->o;
      if(bodyPt == NULL)
	{
	  /* joint followed by joint */
	  distances[jnt_id] = P3D_HUGE;
	}
      else
	{
	  body_id = bodyPt->num;
	  if(bodyPt->np > 0)
	    {
	      /* NEW: */
	      mo1 = p3d_kcd_get_cor_mo(bodyPt->o_id_in_env);
	      /* WAS: */
	      /* mo1 = p3d_kcd_get_cor_mo(bodyPt->o_id); */
	      kcd_get_dist_mo_mo(mo1, &d);	    
	      distances[jnt_id] = d;
	    //  PrintInfo(("dist %.0lf\n,",d));
	    }
	  else
	    {
	      /* empty body */
	      distances[jnt_id] = P3D_HUGE;
	    }
	}
    }
}


/* pre-condition: kcd_collision_exists(DISTANCE_ESTIMATE,min_distance_estimate) */
/*   was called previously                                                      */
/*   returns the latest set of distances computed between the links of the      */
/*   given robot and the static obstacles                                       */
void kcd_get_dist_report_obst(int robot_id, double *distances)
{
  int body_id,jnt_id=0,nof_joints = XYZ_ENV->robot[robot_id]->njoints+1;    
  p3d_obj *bodyPt;
  double d;

  for(jnt_id=0;jnt_id<nof_joints;jnt_id++)
    {
      bodyPt = XYZ_ENV->robot[robot_id]->joints[jnt_id]->o;
      if(bodyPt == NULL)
	{
	  /* joint followed by joint */
	  distances[jnt_id] = P3D_HUGE;
	}
      else
	{
	  body_id = bodyPt->num;
	  if(bodyPt->np > 0)
	    {
	      kcd_get_dist_grp_mo_so(robot_id,body_id,&d);
	      distances[jnt_id] = d;
	    }
	  else
	    {
	      /* empty body */
	      distances[jnt_id] = P3D_HUGE;
	    }
	}
    }
}

/* pre-condition: kcd_collision_exists(DISTANCE_ESTIMATE,min_distance_estimate) */
/*   was called previously                                                      */
/*   returns the latest set of distances computed between the links of the      */
/*   given robot and the static and movable objects                                */
void kcd_get_dist_report_both(int robot_id, double *distances)
{
  int body_id,jnt_id=0,nof_joints = XYZ_ENV->robot[robot_id]->njoints+1;    
  p3d_obj *bodyPt;
  double d_obst, d_body; 
  int mo1;

  for(jnt_id=0;jnt_id<nof_joints;jnt_id++)
    {
      bodyPt = XYZ_ENV->robot[robot_id]->joints[jnt_id]->o;
      if(bodyPt == NULL)
	{
	  /* joint followed by joint */
	  distances[jnt_id] = P3D_HUGE;
	}
      else
	{
	  body_id = bodyPt->num;
	  if(bodyPt->np > 0)
	    {	     
	      /* NEW: */
	      mo1 = p3d_kcd_get_cor_mo(bodyPt->o_id_in_env);
	      /* WAS: */
	      /* mo1 = p3d_kcd_get_cor_mo(bodyPt->o_id); */
	      kcd_get_dist_mo_mo(mo1, &d_body);
	      kcd_get_dist_grp_mo_so(robot_id,body_id,&d_obst);

	      if(d_body < d_obst)
		distances[jnt_id] = d_body;
	      else
		distances[jnt_id] = d_obst;	
	    }
	  else
	    {
	      /* empty body */
	      distances[jnt_id] = P3D_HUGE;
	    }
	}
    }
}



/* Author: Carl Van Geem   Date: 6 November 2001   Function: p3d_kcd_closest_points_mo_environment */

/****************************************************************************
 * Function returns an array of distances  (*distances) computed by the      
 * collision detector between  a given mov.obj. and the environment
 *
 * In :  mov.obj. pointer     
 * Out : distance   is distance estimate between mov.obj. and closest so
 *       point_mo[3] coordinates of the point of the mov.obj. (mo) that
 *                        is closest to the so  (relative to the origine of the
 *                        coordinate frame (=environment))
 *       point_so[3] coordinates of the point of the object (so) that
 *                        is closest to the mo (relative to the origine of the
 *                        coordinate frame)
 *
 ****************************************************************************/
void p3d_kcd_closest_points_mo_environment(p3d_obj *mobjPt, p3d_vector3 point_mo, 
					     p3d_vector3 point_so, double *distance)
{
  int mo1;

  mo1 = p3d_kcd_get_cor_mo(mobjPt->o_id_in_env);
  kcd_get_points_closest_pair_mo_so(mo1,point_mo,point_so);
  kcd_get_dist_mo_so(mo1,distance);
}

/* Author: Carl Van Geem   Date: 6 November 2001   Function: p3d_kcd_closest_points_mo_of_both */

/*! 
\brief get the real closest points( test is made between closest points to moving objects
and closest points to static objects)
\param distance is distance estimate between body i and closest so
\param point_mo[3] coordinates of the point of the mov.obj. (mo) that
                        is closest to the so  (relative to the origine of the
                        coordinate frame (=environment))
\param point_o[3] coordinates of the point of the object (o) that
                        is closest to the mo (relative to the origine of the
                        coordinate frame)
*/
void p3d_kcd_closest_points_mo_of_both(p3d_obj *mobjPt, p3d_vector3 point_mo, 
				    p3d_vector3 point_o, double *distance)
{
  int mo1,mo2;
  double d_body,d_obst;

  mo1 = p3d_kcd_get_cor_mo(mobjPt->o_id_in_env);
  kcd_get_dist_mo_mo(mo1, &d_body);
  kcd_get_dist_mo_so(mo1, &d_obst);

  if (d_body < d_obst) {
    *distance = d_body;
    kcd_get_points_closest_pair_mos(mo1,&mo2,point_mo,point_o);
  } else {
    *distance = d_obst;	 
    kcd_get_points_closest_pair_mo_so(mo1,point_mo,point_o);
  }
}

/* ************************************************************ *
 * ************************************************************ *
 * 
 * new functions allowing to add an obstacle dynamically to KCD
 * 
 * ************************************************************ *
 * ************************************************************ */

int kcd_add_obstacle(p3d_obj *obst)
{
  int nof_prims;
  int nof_obsts = 1;
  int current_scene = -1;
  int p3d_env_id;
  p3d_poly *primPt;
  int kcdPrimId;     /* not stored anyway */
  int kcdObstId;
  int found;
  int i;
  int tot_nof_objs;
  
  if(obst == NULL) 
    goto error;

  nof_prims = obst->np;
  // current_scene = ...;
  p3d_env_id = XYZ_ENV->num;
  found = FALSE;
  for(i=0;(i<nof_p3d_kcd_envs)&&(!found);i++)
    {
      if( p3d_kcd_env_arr[i].p3d_env_id == p3d_env_id )
	{
	  current_scene = p3d_kcd_env_arr[i].kcd_scene_id;
	  found = TRUE;
	}
    }
  if(!found) 
    goto error;

  kcd_addObst_beg_scene(current_scene,nof_prims,nof_obsts);
  kcd_addObst_beg_obj();
  for(i=0;i<nof_prims;i++)
    {
      primPt = obst->pol[i];
      kcdPrimId = kcd_addObst_add_prim(primPt);
    }
  kcdObstId = kcd_addObst_end_obj();
  kcd_addObst_end_scene();

  /* modify tables */
  tot_nof_objs = p3d_kcd_env_arr[current_scene].tot_nof_objs;

  p3d_to_kcd_array = MY_REALLOC(p3d_to_kcd_array,p3d_to_kcd,tot_nof_objs,tot_nof_objs+1);
  o_id_to_obj_ptr = MY_REALLOC(o_id_to_obj_ptr,pp3d_obj,tot_nof_objs,tot_nof_objs+1); 
  o_id_to_input_i = MY_REALLOC(o_id_to_input_i,int,tot_nof_objs,tot_nof_objs+1);      

  p3d_to_kcd_array[kcdObstId].p3d_o_id = obst->o_id_in_env;
  p3d_to_kcd_array[kcdObstId].kcd_obj_id = kcdObstId;
  p3d_to_kcd_array[kcdObstId].is_movable = FALSE;
  o_id_to_obj_ptr[obst->o_id_in_env] = obst;
  // WAS:
  // o_id_to_input_i[tot_nof_objs] = tot_nof_objs; /* since it is the latest object added to KCD*/
  // NOW:
  o_id_to_input_i[obst->o_id_in_env] = kcdObstId;

  p3d_kcd_env_arr[current_scene].tot_nof_objs += 1;
  p3d_kcd_env_arr[current_scene].p3d_to_kcd_array = p3d_to_kcd_array;
  p3d_kcd_env_arr[current_scene].o_id_to_obj_ptr = o_id_to_obj_ptr;
  p3d_kcd_env_arr[current_scene].o_id_to_input_i = o_id_to_input_i;

  return TRUE;
 error:
  return FALSE;
}


int kcd_add_obstacle_list(p3d_obj **obstList, int nof_obsts)
{
  int nof_prims;
  int current_scene = -1;
  int p3d_env_id;
  p3d_poly *primPt;
  int kcdPrimId;     /* not stored anyway */
  int *kcdObstId = NULL;
  int found;
  int i,j;
  int tot_nof_objs;
  p3d_obj *obst = NULL;
  
  if(obstList == NULL) 
    goto error;

  nof_prims = 0;
  for(j=0;j<nof_obsts;j++)
    {
      obst = obstList[j];
      if(obst == NULL)
	goto error;
    
      nof_prims += obst->np;
    }
  kcdObstId = MY_ALLOC(int,nof_obsts);

  // current_scene = ...;
  p3d_env_id = XYZ_ENV->num;
  found = FALSE;
  for(i=0;(i<nof_p3d_kcd_envs)&&(!found);i++)
    {
      if( p3d_kcd_env_arr[i].p3d_env_id == p3d_env_id )
	{
	  current_scene = p3d_kcd_env_arr[i].kcd_scene_id;
	  found = TRUE;
	}
    }
  if(!found) 
    goto error;

  kcd_addObst_beg_scene(current_scene,nof_prims,nof_obsts);
  for(j=0;j<nof_obsts;j++)
    {
      obst = obstList[j];
      kcd_addObst_beg_obj();
      for(i=0;i<obst->np;i++)
	{
	  primPt = obst->pol[i];
	  kcdPrimId = kcd_addObst_add_prim(primPt);
	}
      kcdObstId[j] = kcd_addObst_end_obj();
    }
  kcd_addObst_end_scene();

  /* modify tables */
  tot_nof_objs = p3d_kcd_env_arr[current_scene].tot_nof_objs;

  p3d_to_kcd_array = MY_REALLOC(p3d_to_kcd_array,p3d_to_kcd,tot_nof_objs,tot_nof_objs+nof_obsts);
  o_id_to_obj_ptr = MY_REALLOC(o_id_to_obj_ptr,pp3d_obj,tot_nof_objs,tot_nof_objs+nof_obsts); 
  o_id_to_input_i = MY_REALLOC(o_id_to_input_i,int,tot_nof_objs,tot_nof_objs+nof_obsts);      
  
  for(j=0;j<nof_obsts;j++)
    {
      obst = obstList[j];
      p3d_to_kcd_array[kcdObstId[j]].p3d_o_id = obst->o_id_in_env;
      p3d_to_kcd_array[kcdObstId[j]].kcd_obj_id = kcdObstId[j];
      p3d_to_kcd_array[kcdObstId[j]].is_movable = FALSE;
      o_id_to_obj_ptr[obst->o_id_in_env] = obst;
      // WAS:
      // o_id_to_input_i[tot_nof_objs+j] = tot_nof_objs+j;
      // NEW:
      o_id_to_input_i[obst->o_id_in_env] = kcdObstId[j]; 
    }

  p3d_kcd_env_arr[current_scene].tot_nof_objs += nof_obsts;
  p3d_kcd_env_arr[current_scene].p3d_to_kcd_array = p3d_to_kcd_array;
  p3d_kcd_env_arr[current_scene].o_id_to_obj_ptr = o_id_to_obj_ptr;
  p3d_kcd_env_arr[current_scene].o_id_to_input_i = o_id_to_input_i;

  MY_FREE(kcdObstId,int,nof_obsts);
  return TRUE;
 error:
  return FALSE;
}

//! @ingroup kcd
//! Prints the names the two objects that were reported as colliding during
//! the last collision test. This function must be called after a positive collision test.
void kcd_print_colliding_pair()
{
  p3d_obj *o1Pt, *o2Pt;
  p3d_col_test_choice();
  p3d_kcd_get_pairObjInCollision ( &o1Pt, &o2Pt );
  printf("Collision between \"%s\" and \"%s\" \n", o1Pt->name, o2Pt->name);
}
