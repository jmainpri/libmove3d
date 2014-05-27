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
#include "Collision-pkg.h"

/* types */
typedef int *intp;

typedef struct robot_vs_static_aabb_collision
{
  int robot_id;
  int link_id;             /* for link treatment */
  int place_link;          /* for link treatment */
  kcd_bb robot_box;        /* contains AABB around link !! */
  int static_object_bb_id;    
  int hier_level;
  int bb_id_at_level;
  int some_obb_collides;   /* TRUE iff. some leaf AABB contains colliding OBB */
} robot_vs_static_aabb_collision, *robot_vs_static_aabb_collision_p;

typedef robot_vs_static_aabb_collision *list_of_robot_vs_static_aabb_collision;

/* global variables used in this file only */
/* static robot_vs_static_aabb_collision *res = NULL;*/

static robot_vs_static_aabb_collision **interm_res = NULL;
static int *nof_interm_res = NULL;

list_of_robot_vs_static_aabb_collision *robots_vs_static_aabbs_in_col = NULL;
int *nof_robots_vs_static_aabbs = NULL;

kcd_aabb_lists AABB_TREE;
int nr_kcd_aabb_list;
int *nr_aabb_hier_el;


/* ******************************************************************************* * 
 * Function :   kcd_get_mo_aabb()
 * returns AABB around movable object:
 *  - computed by external user-defined method linked to in kcd_get_aabb_on_mo()
 *    OR computed by KCD itself
 *  - the KCD function get_kcd_bb_aabb() only does a look-up in all_bbs[] 
 * AGRS IN  :   kcd_ext_o           id of movable object
 *              kcd_bb              id of bb around movable object
 * ARGS OUT :   x1,x2,y1,y2,z1,z2   AABB box extremes
 * ******************************************************************************* */
static void kcd_get_mo_aabb(int kcd_ext_o, int kcd_bb,
			    double *x1,double *x2,double *y1,
			    double *y2,double *z1,double *z2)
{
  
  if(! kcd_get_aabb_on_mo(kcd_ext_o,x1,x2,y1,y2,z1,z2) )
    {
      /* call function of KCD to recuperate x1,x2,y1,y2,z1,z2 */
      get_kcd_bb_aabb(kcd_bb,x1,x2,y1,y2,z1,z2);
    }
}


/* global variables defining the AABB hierarchy */
void kcd_init_AABB_TREE()
{
   AABB_TREE = NULL;
   nr_kcd_aabb_list = 0;
   nr_aabb_hier_el = NULL;
}

void kcd_clean_up_AABB_TREE()
{
  int i,j,k,group_below_nr,nr_grps;

  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      nr_grps = nr_aabb_hier_el[i];
      for(j=0;j<nr_grps;j++)
	{
	  group_below_nr = AABB_TREE[i][j].group_below_nr;
	  for(k=0;k<2*group_below_nr;k++)
	    {
	      MY_FREE(AABB_TREE[i][j].aabb_xlist[k],axis_list_el,1);
	      MY_FREE(AABB_TREE[i][j].aabb_ylist[k],axis_list_el,1);
	      MY_FREE(AABB_TREE[i][j].aabb_zlist[k],axis_list_el,1);
	    }
	  if(group_below_nr > 0)
	    {
	      MY_FREE(AABB_TREE[i][j].group_below_aabb_hier_el,int,group_below_nr);
	      MY_FREE(AABB_TREE[i][j].aabb_xlist,axis_list_elp,2*group_below_nr);
	      MY_FREE(AABB_TREE[i][j].aabb_ylist,axis_list_elp,2*group_below_nr);
	      MY_FREE(AABB_TREE[i][j].aabb_zlist,axis_list_elp,2*group_below_nr);
	    }
	}
      MY_FREE(AABB_TREE[i],aabb_hier_el,nr_aabb_hier_el[i]);
    }

  MY_FREE(AABB_TREE,kcd_aabb_list,nr_kcd_aabb_list);
  MY_FREE(nr_aabb_hier_el,int,nr_kcd_aabb_list);
  AABB_TREE = NULL;
  nr_kcd_aabb_list = 0;
  nr_aabb_hier_el = NULL;
}

/* not static: */
int kcd_get_the_bb(int val1, int val2)
{
  return AABB_TREE[val1][val2].the_bb;
}

static int kcd_empty_volume_small_after_union(int hierheight,int box_i,int box_j)
{
  int is_small = TRUE;
  /* we are looking at boxes: */
  kcd_bb *aabb1 = all_bbs[AABB_TREE[hierheight][box_i].the_bb];
  kcd_bb *aabb2 = all_bbs[AABB_TREE[hierheight][box_j].the_bb];
  double V1, V2,V;
  double xm,xM,ym,yM,zm,zM;

  V1 = (aabb1->x2 - aabb1->x1)*(aabb1->y2 - aabb1->y1)*(aabb1->z2 - aabb1->z1);
  V2 = (aabb2->x2 - aabb2->x1)*(aabb2->y2 - aabb2->y1)*(aabb2->z2 - aabb2->z1);

  if(V1 < 0.0)
    PrintInfo(("NEGATIVE V1=%f\n",V1));
  if(V2 < 0.0)
    PrintInfo(("NEGATIVE V2=%f\n",V2));

  if(aabb1->x1 < aabb2->x1)
    xm = aabb1->x1;
  else
    xm = aabb2->x1;
  if(aabb1->x2 < aabb2->x2)
    xM = aabb2->x2;
  else
    xM = aabb1->x2;
  if(aabb1->y1 < aabb2->y1)
    ym = aabb1->y1;
  else
    ym = aabb2->y1;
  if(aabb1->y2 < aabb2->y2)
    yM = aabb2->y2;
  else
    yM = aabb1->y2;
  if(aabb1->z1 < aabb2->z1)
    zm = aabb1->z1;
  else
    zm = aabb2->z1;
  if(aabb1->z2 < aabb2->z2)
    zM = aabb2->z2;
  else
    zM = aabb1->z2;

  V = (xM - xm)*(yM - ym)*(zM - zm);
  if(V  < 0.0)
    PrintInfo(("NEGATIVE V =%f\n",V ));

  is_small = ( V < 1.0 * (V1 + V2) ); /* works better for grandesalle.p3d */
  is_small = TRUE;   /* works best for edf.p3d */
  return is_small;
}



/* this function might be unused */
/* to activate this function remove some comments 
  static int kcd_object_not_too_big(int box_nr,int obox_nr)
  {
  int resul = TRUE;
  kcd_bb *obj_aabb = all_bbs[box_nr];
  double x,y,z,env_size[3], box_size[3], size_factor = 3.1;
*/
/* compute size of environment */
/*  kcd_get_scene_size(&x,&y,&z);
    env_size[0] = x; 
    env_size[1] = y;   
    env_size[2] = z;*/
/* get size of AABB around the object */
/*  box_size[0] = obj_aabb->x2 - obj_aabb->x1;
    box_size[1] = obj_aabb->y2 - obj_aabb->y1;
    box_size[2] = obj_aabb->z2 - obj_aabb->z1;*/
/* compare sizes */
/*  if(  ( (size_factor * box_size[0]) > env_size[0] )  ||
    ( (size_factor * box_size[1]) > env_size[1] )  ||
    ( (size_factor * box_size[2]) > env_size[2] )   )
    {
    resul = FALSE;
    }
    else
    {*/
/* go ahead with second box */
/* obj_aabb = all_bbs[obox_nr];
   box_size[0] = obj_aabb->x2 - obj_aabb->x1;
   box_size[1] = obj_aabb->y2 - obj_aabb->y1;
   box_size[2] = obj_aabb->z2 - obj_aabb->z1;*/
/* compare sizes */
/*      if(  ( (size_factor * box_size[0]) > env_size[0] )  ||
	( (size_factor * box_size[1]) > env_size[1] )  ||
	( (size_factor * box_size[2]) > env_size[2] )   )
	{
	resul = FALSE;
	}
	}
	return resul;
	}
*/


/* report on number of levels in the hierarchy */
int kcd_get_total_height(void)
{
  return nr_kcd_aabb_list;
}

/* report on the number of AABBs in a level of the hierarchy */
int kcd_get_nofels_on_level(int hierheight)
{
  return nr_aabb_hier_el[hierheight];
}

/* return AABB nr "i" in level "hierheight" of the hierarchy */
int kcd_get_aabb_i_by_level(int hierheight,int i)
{
  return AABB_TREE[hierheight][i].the_bb;
}

/* ****************************************************************************************** *
 * Function: kcd_determine_aabb_around_aabbs()
 * compute an aabb around a group of aabbs in a lower level of the AABB_TREE
 * ARGS IN:  the_aabb_group         array of indices of the group of AABBs on the lower level
 *           the_aabb_group_nofels  number of elements in array  the_aabb_group
 *           hierheight             level (height in the AABB_TREE hierarchy) on which the
 *                                  AABBs in  the_aabb_group  are stored
 * ****************************************************************************************** */
int kcd_determine_aabb_around_aabbs(int *the_aabb_group, int the_aabb_group_nofels, int hierheight)
{
  int i,res=0;
  double mx1,mx2,my1,my2,mz1,mz2;
  double x1,x2,y1,y2,z1,z2;
  /* B Kineo Carl 27.02.2002 */
  int ext_obj_id = -1;
  /* E Kineo Carl 27.02.2002 */

  if(the_aabb_group_nofels>0)
    {
      mx1=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->x1;
      mx2=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->x2;
      my1=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->y1;
      my2=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->y2;
      mz1=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->z1;
      mz2=all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->z2;
      /* B Kineo Carl 27.02.2002 */
      ext_obj_id = all_bbs[AABB_TREE[hierheight][the_aabb_group[0]].the_bb]->ext_obj_id;
      /* E Kineo Carl 27.02.2002 */

      for(i=1;i<the_aabb_group_nofels;i++)
	{
	  x1=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->x1;
	  x2=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->x2;
	  y1=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->y1;
	  y2=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->y2;
	  z1=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->z1;
	  z2=all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->z2;
 	  if(x1<mx1)
	    mx1=x1;
 	  if(x2>mx2)
	    mx2=x2;
 	  if(y1<my1)
	    my1=y1;
 	  if(y2>my2)
	    my2=y2;
 	  if(z1<mz1)
	    mz1=z1;
 	  if(z2>mz2)
	    mz2=z2;

      /* B Kineo Carl 27.02.2002 */
	  if(ext_obj_id != all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->ext_obj_id)
	    ext_obj_id = -1;
      /* E Kineo Carl 27.02.2002 */

	}
      res = put_new_aabb_away(ext_obj_id,FALSE,mx1,mx2,my1,my2,mz1,mz2, the_aabb_group, the_aabb_group_nofels, hierheight);
    }
  else
    {
      PrintInfo(("error: group of 0 elements\n"));
    }
  return res;
}

double new_aabbs_square_distance(int b1_nr,int b2_nr)
{
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];
  double lx=0.0,ly=0.0,lz=0.0;
  double dist_x_s1 = ABS(aabb1->x2 - aabb2->x1);
  double dist_x_s2 = ABS(aabb2->x2 - aabb1->x1);
  double dist_y_s1 = ABS(aabb1->y2 - aabb2->y1);
  double dist_y_s2 = ABS(aabb2->y2 - aabb1->y1);
  double dist_z_s1 = ABS(aabb1->z2 - aabb2->z1);
  double dist_z_s2 = ABS(aabb2->z2 - aabb1->z1);

  if(aabb1->x2 - aabb1->x1 + aabb2->x2 - aabb2->x1 < MAX(dist_x_s1,dist_x_s2))
    lx = MIN(dist_x_s1,dist_x_s2);
  
  if(aabb1->y2 - aabb1->y1 + aabb2->y2 - aabb2->y1 < MAX(dist_y_s1,dist_y_s2))
    ly = MIN(dist_y_s1,dist_y_s2);

  if(aabb1->z2 - aabb1->z1 + aabb2->z2 - aabb2->z1 < MAX(dist_z_s1,dist_z_s2))
    lz = MIN(dist_z_s1,dist_z_s2);

  return ( lx*lx + ly*ly + lz*lz );
}

/* ****************************************************************************************** *
 * Function: aabbs_overlap()
 *           determine whether two aabb's overlap (returns true or false) 
 * ****************************************************************************************** */
int aabbs_overlap(kcd_bb *aabb1,kcd_bb *aabb2)
{
  if (aabb1->x1 < aabb2->x1)
    {
      if (aabb1->x2 < aabb2->x1)
	return FALSE;
    }
  else
    {
      if (aabb1->x1 > aabb2->x2)
	return FALSE;
    }
  if (aabb1->y1 < aabb2->y1)
    {
      if (aabb1->y2 < aabb2->y1)
	return FALSE;
    }
  else
    {
      if (aabb1->y1 > aabb2->y2)
	return FALSE;
    }
  if (aabb1->z1 < aabb2->z2)
    {
      if (aabb1->z2 < aabb2->z1)
	return FALSE;
    }
  else
    {
      if (aabb1->z1 > aabb2->z2)
	return FALSE;
    }
  /* PrintInfo(("aabbs_overlap returns TRUE\n")); */
  return TRUE;
}

/* ****************************************************************************************** *
 * Function: aabbs_square_distance()
 *           determine whether two aabb's overlap and 
 * ARGS IN : b1_nr, b2_nr     indices of two bounding boxes in data structure  all_bbs
 * Return  : square distance between the aabb's when they do not intersect,
 *           otherwise return 0.0 
 * ****************************************************************************************** */
double aabbs_square_distance(int b1_nr,int b2_nr)
{
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];
  double lx=0.0,ly=0.0,lz=0.0;

  if (aabb1->x1 < aabb2->x1)
    {
      if (aabb1->x2 < aabb2->x1)
	{
	  lx = aabb1->x2 - aabb2->x1;
	}
    }
  else
    {
      if (aabb1->x1 > aabb2->x2)
	{
	  lx = aabb2->x2 - aabb1->x1;
	}
    }
  if (aabb1->y1 < aabb2->y1)
    {
      if (aabb1->y2 < aabb2->y1)
	{
	  ly = aabb1->y2 - aabb2->y1;
	}
    }
  else
    {
      if (aabb1->y1 > aabb2->y2)
	{
	  ly = aabb2->y2 - aabb1->y1;
	}
    }
  if (aabb1->z1 < aabb2->z2)
    {
      if (aabb1->z2 < aabb2->z1)
	{
	  lz = aabb1->z2 - aabb2->z1;
	}
    }
  else
    {
      if (aabb1->z1 > aabb2->z2)
	{
	  lz = aabb2->z2 - aabb1->z1;
	}
    }
  /* they intersect <==> lx==0 && ly==0 && lz==0 */
  return  (lx*lx + ly*ly + lz*lz) ;
}

/* ****************************************************************************************** *
 * Function : aabbs_distance()
 *            determine whether two aabb's overlap and 
 * ARGS IN :  b1_nr, b2_nr     indices of two bounding boxes in data structure  all_bbs
 * Returns :  distance between the aabb's when they do not intersect,
 *            otherwise return 0.0 
 * ****************************************************************************************** */
double aabbs_distance(int b1_nr, int b2_nr)
{
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];
  double lx=0.0,ly=0.0,lz=0.0;

  if (LNEQ(aabb1->x1,aabb2->x1))
    {
      if (LNEQ(aabb1->x2,aabb2->x1))
	{
	  lx = aabb1->x2 - aabb2->x1;
	}
    }
  else
    {
      if (GNEQ(aabb1->x1,aabb2->x2))
	{
	  lx = aabb2->x2 - aabb1->x1;
	}
    }
  if (LNEQ(aabb1->y1,aabb2->y1))
    {
      if (LNEQ(aabb1->y2,aabb2->y1))
	{
	  ly = aabb1->y2 - aabb2->y1;
	}
    }
  else
    {
      if (GNEQ(aabb1->y1,aabb2->y2))
	{
	  ly = aabb2->y2 - aabb1->y1;
	}
    }
  if (LNEQ(aabb1->z1,aabb2->z2))
    {
      if (LNEQ(aabb1->z2,aabb2->z1))
	{
	  lz = aabb1->z2 - aabb2->z1;
	}
    }
  else
    {
      if (GNEQ(aabb1->z1,aabb2->z2))
	{
	  lz = aabb2->z2 - aabb1->z1;
	}
    }
  /* they intersect <==> lx==0 && ly==0 && lz==0 */
  return sqrt( lx*lx + ly*ly + lz*lz );
}

/* ****************************************************************************************** *
 * Function :   aabbs_penetration_measure()
 * ARGS IN :  b1_nr, b2_nr     indices of two bounding boxes in data structure  all_bbs
 * No PRECOND: returns 0.0 if boxes don't overlap 
 * determine how much two aabb's overlap (returns penetration measure)
 * penetration measure === diagonal of intersection of the two AABBs  
 * (NOTE: intersection of two AABBs is an AABB) 
 * ****************************************************************************************** */
double aabbs_penetration_measure(int b1_nr, int b2_nr)
{
  double lx,ly,lz;
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];

  /* check for intersection */
  if (aabb1->x1 < aabb2->x1)
    {
      if (aabb1->x2 < aabb2->x1)
	return 0.0;
    }
  else
    {
      if (aabb1->x1 > aabb2->x2)
	return 0.0;
    }
  if (aabb1->y1 < aabb2->y1)
    {
      if (aabb1->y2 < aabb2->y1)
	return 0.0;
    }
  else
    {
      if (aabb1->y1 > aabb2->y2)
	return 0.0;
    }
  if (aabb1->z1 < aabb2->z2)
    {
      if (aabb1->z2 < aabb2->z1)
	return 0.0;
    }
  else
    {
      if (aabb1->z1 > aabb2->z2)
	return 0.0;
    }

  /* they intersect: compute diagonal of the intersection */
  if(aabb1->x1 < aabb2->x1)
    if(aabb1->x2 < aabb2->x2)
      lx = aabb2->x1 - aabb1->x2;
    else 
      lx = aabb2->x1 - aabb2->x2;
  else
    if(aabb1->x2 < aabb2->x2)
      lx = aabb1->x1 - aabb1->x2;
    else 
      lx = aabb1->x1 - aabb2->x2;

  if(aabb1->y1 < aabb2->y1)
    if(aabb1->y2 < aabb2->y2)
      ly = aabb2->y1 - aabb1->y2;
    else 
      ly = aabb2->y1 - aabb2->y2;
  else
    if(aabb1->y2 < aabb2->y2)
      ly = aabb1->y1 - aabb1->y2;
    else 
      ly = aabb1->y1 - aabb2->y2;

  if(aabb1->z1 < aabb2->z1)
    if(aabb1->z2 < aabb2->z2)
      lz = aabb2->z1 - aabb1->z2;
    else 
      lz = aabb2->z1 - aabb2->z2;
  else
    if(aabb1->z2 < aabb2->z2)
      lz = aabb1->z1 - aabb1->z2;
    else 
      lz = aabb1->z1 - aabb2->z2;

  return sqrt( lx*lx + ly*ly + lz*lz )  ;
}

/* ****************************************************************************************** *
 * Function : aabbs_square_penetration_measure_when_collision()
 * ARGS IN :  b1_nr, b2_nr     indices of two bounding boxes in data structure  all_bbs
 * PRECOND  : aabb1 and aabb2 overlap
 *  determine how much two aabb's overlap (returns penetration measure)
 *  penetration measure === diagonal of intersection of the two AABBs  
 *  (NOTE: intersection of two AABBs is an AABB) 
 * ****************************************************************************************** */
double aabbs_square_penetration_measure_when_collision(int b1_nr, int b2_nr)
{
  double lx,ly,lz;
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];

  /* don't check for intersection */

  /* one is inside the other (inclusion) */
  if( ( ((LEQ(aabb1->x1,aabb2->x1))&&(LEQ(aabb2->x2,aabb1->x2))) &&
	((LEQ(aabb1->y1,aabb2->y1))&&(LEQ(aabb2->y2,aabb1->y2))) &&
	((LEQ(aabb1->z1,aabb2->z1))&&(LEQ(aabb2->z2,aabb1->z2))) ) || 
      ( ((LEQ(aabb2->x1,aabb1->x1))&&(LEQ(aabb1->x2,aabb2->x2))) &&
	((LEQ(aabb2->y1,aabb1->y1))&&(LEQ(aabb1->y2,aabb2->y2))) &&
	((LEQ(aabb2->z1,aabb1->z1))&&(LEQ(aabb1->z2,aabb2->z2))) ) )
    {
      /* inclusion */
      lx=MAX(aabb1->x2-aabb2->x1,aabb2->x2-aabb1->x1);
      ly=MAX(aabb1->y2-aabb2->y1,aabb2->y2-aabb1->y1);
      lz=MAX(aabb1->z2-aabb2->z1,aabb2->z2-aabb1->z1);
    }
  else
    {
      /* they intersect: compute diagonal of the intersection */
      if(EQ(aabb1->x1,aabb2->x1))
	lx = ABS(aabb1->x2 - aabb2->x2);
      else if(aabb1->x1 < aabb2->x1)
	lx = aabb1->x2 - aabb2->x1;
      else
	lx = aabb2->x2 - aabb1->x1;

      if(EQ(aabb1->y1,aabb2->y1))
	ly = ABS(aabb1->y2 - aabb2->y2);
      else if(aabb1->y1 < aabb2->y1)
	ly = aabb1->y2 - aabb2->y1;
      else
	ly = aabb2->y2 - aabb1->y1;
      
      if(EQ(aabb1->z1,aabb2->z1))
	lz = ABS(aabb1->z2 - aabb2->z2);
      else if(aabb1->z1 < aabb2->z1)
	lz = aabb1->z2 - aabb2->z1;
      else
	lz = aabb2->z2 - aabb1->z1;
    }
  return ( lx*lx + ly*ly + lz*lz )  ;
}

/* FOR STATIC OBJECTS */
void kcd_sort_aabb_tree(axis_list_elp *the_list, int the_list_size)
{
  double v;
  int i,j,current_pos;
  axis_list_elp help_elp,help_elp_j;

  for(i=1;i<the_list_size;i++)
    {
      current_pos = i;
      v = the_list[current_pos]->val;
      
      for(j=i-1;j>=0;j--)
	{
	  if(the_list[j]->val < v)
	    {
	      /* switch places */
	      help_elp   = the_list[current_pos];
	      help_elp_j = the_list[j];
	      the_list[current_pos] = help_elp_j;
	      the_list[j] = help_elp;
	      current_pos = j;
	    }
	}
    }
}

/* ****************************************************************************************** *
 * Function :  insert_static_aabb()
 *             insert an aabb in the three lists on lowest level 
 * ARGS IN  :  aabbtoinsert_nr    index in all_bbs of AABB to insert in AABB_TREE
 *             hierheight         level on which to insert in AABB_TREE
 *             group_nr           index at level  hierheight  of the group that
 *                                the AABB encloses
 *             group_below        array of bounding boxes around which the AABB is computed
 *             group_below_nr     number of elements in the array  group_below
 * ****************************************************************************************** */
void insert_static_aabb(int aabbtoinsert_nr, int hierheight, int group_nr, int *group_below, int group_below_nr)
{
  int i,ch;
  /* B 12.03.2002 */
  int dd;
  /* E 12.03.2002 */

  if(AABB_TREE == NULL)
    {
      AABB_TREE = MY_ALLOC(kcd_aabb_list,1);
      AABB_TREE[hierheight]=NULL;
      nr_kcd_aabb_list = 1;
      nr_aabb_hier_el = MY_ALLOC(int,1);
      nr_aabb_hier_el[hierheight] = 0;
    }

  if(hierheight > 0)
    {
      if(nr_kcd_aabb_list <= hierheight)
	{
	  AABB_TREE = MY_REALLOC(AABB_TREE,kcd_aabb_list,nr_kcd_aabb_list,hierheight+1);
	  nr_aabb_hier_el = MY_REALLOC(nr_aabb_hier_el,int,nr_kcd_aabb_list,hierheight+1);
	  nr_kcd_aabb_list = hierheight+1;
	  AABB_TREE[hierheight]=NULL;
	  nr_aabb_hier_el[hierheight] = 0;
	}
    }
     
  if(nr_aabb_hier_el[hierheight] != group_nr+1)  /* modif. Carl 20 Oct. 2000, was <= */
    {
      AABB_TREE[hierheight]=MY_REALLOC(AABB_TREE[hierheight],
				       aabb_hier_el,nr_aabb_hier_el[hierheight],group_nr+1);
      nr_aabb_hier_el[hierheight] = group_nr+1;
      /* B 12.03.2002 */
      AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el = NULL;
      AABB_TREE[hierheight][group_nr].group_below_nr = 0;
      AABB_TREE[hierheight][group_nr].aabb_xlist_nofels = 0;
      AABB_TREE[hierheight][group_nr].aabb_ylist_nofels = 0;
      AABB_TREE[hierheight][group_nr].aabb_zlist_nofels = 0;
      AABB_TREE[hierheight][group_nr].aabb_xlist = NULL;
      AABB_TREE[hierheight][group_nr].aabb_ylist = NULL;
      AABB_TREE[hierheight][group_nr].aabb_zlist = NULL;
      /* E 12.03.2002 */
    }

  AABB_TREE[hierheight][group_nr].the_bb = aabbtoinsert_nr;
  /* B 12.03.2002 */
/*   AABB_TREE[hierheight][group_nr].group_below_nr = group_below_nr;
 */
/*   AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el = NULL;
 */
  /* E 12.03.2002 */
  AABB_TREE[hierheight][group_nr].this_hier_level = hierheight;
  AABB_TREE[hierheight][group_nr].this_place_at_level = group_nr;

  all_bbs[aabbtoinsert_nr]->hier_level = hierheight;
  all_bbs[aabbtoinsert_nr]->place_at_level = group_nr;

  if(group_below_nr > 0)
    {
      /* B 12.03.2002 */
/*       AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el = MY_ALLOC(int,group_below_nr);
 */
      AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el = 
	MY_REALLOC(AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el,int,
		   AABB_TREE[hierheight][group_nr].group_below_nr,group_below_nr);
      /* E 12.03.2002 */
      for(ch=0;ch<group_below_nr;ch++)
	{
	  AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el[ch] = group_below[ch];
	}
      /* X Y Z lists */
      if(group_below_nr > 0)
	/* B 12.03.2002 */
	{
/* 	  AABB_TREE[hierheight][group_nr].aabb_xlist
 */
/* 	    = MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
	  AABB_TREE[hierheight][group_nr].aabb_xlist
	    = MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_xlist,axis_list_elp,
			 2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
	  for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	    AABB_TREE[hierheight][group_nr].aabb_xlist[dd] = NULL;
	}
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_xlist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_xlist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_xlist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_xlist[i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_xlist[i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->x1;
	  
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->x2;
	}
      
      if(group_below_nr > 0)
	/* B 12.03.2002 */
	{
/* 	  AABB_TREE[hierheight][group_nr].aabb_ylist
 */
/* 	    = MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
	  AABB_TREE[hierheight][group_nr].aabb_ylist
	    = MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_ylist,axis_list_elp,
			 2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
	  for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	    AABB_TREE[hierheight][group_nr].aabb_ylist[dd] = NULL;	  
	}
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_ylist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_ylist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_ylist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_ylist[i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_ylist[i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->y1;
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->y2;
	}
      
      if(group_below_nr > 0)
	/* B 12.03.2002 */
	{
/* 	  AABB_TREE[hierheight][group_nr].aabb_zlist
 */
/* 	    = MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
	  AABB_TREE[hierheight][group_nr].aabb_zlist
	    = MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_zlist,axis_list_elp,
			 2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
	  for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	    AABB_TREE[hierheight][group_nr].aabb_zlist[dd] = NULL;
	}
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_zlist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_zlist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_zlist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_zlist[i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_zlist[i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->z1;
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i]->the_bb = AABB_TREE[hierheight-1][group_below[i]].the_bb;
	  AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i]->val = all_bbs[AABB_TREE[hierheight-1][group_below[i]].the_bb]->z2;
	}
    }
  else
    {
      /* AABB HIER LEVEL: group_below_nr == 0 */
      /* X Y Z lists */
      /* B 12.03.2002 */
/*       AABB_TREE[hierheight][group_nr].aabb_xlist
 */
/* 	= MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
      AABB_TREE[hierheight][group_nr].aabb_xlist
	= MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_xlist,axis_list_elp,
		     2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
      for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	AABB_TREE[hierheight][group_nr].aabb_xlist[dd] = NULL;
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_xlist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_xlist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_xlist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_xlist[i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_xlist[i]->val = all_bbs[group_below[i]]->x1;
	  
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_xlist[group_below_nr+i]->val = all_bbs[group_below[i]]->x2;
	}
      
      /* B 12.03.2002 */
/*       AABB_TREE[hierheight][group_nr].aabb_ylist
 */
/* 	= MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
      AABB_TREE[hierheight][group_nr].aabb_ylist
	= MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_ylist,axis_list_elp,
		     2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
      for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	AABB_TREE[hierheight][group_nr].aabb_ylist[dd] = NULL;
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_ylist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_ylist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_ylist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_ylist[i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_ylist[i]->val = all_bbs[group_below[i]]->y1;
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_ylist[group_below_nr+i]->val = all_bbs[group_below[i]]->y2;
	}
      
      /* B 12.03.2002 */
/*       AABB_TREE[hierheight][group_nr].aabb_zlist
 */
/* 	= MY_ALLOC(axis_list_elp,2*group_below_nr);
 */
      AABB_TREE[hierheight][group_nr].aabb_zlist
	= MY_REALLOC(AABB_TREE[hierheight][group_nr].aabb_zlist,axis_list_elp,
		     2*AABB_TREE[hierheight][group_nr].group_below_nr,2*group_below_nr);
      for(dd = 2*AABB_TREE[hierheight][group_nr].group_below_nr; dd < 2*group_below_nr; dd++)
	AABB_TREE[hierheight][group_nr].aabb_zlist[dd] = NULL;
      /* E 12.03.2002 */
      AABB_TREE[hierheight][group_nr].aabb_zlist_nofels = 2*group_below_nr;
      for(i=0;i<group_below_nr;i++)
	{
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_zlist[i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_zlist[i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_zlist[i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_zlist[i]->val = all_bbs[group_below[i]]->z1;
	  /* B 12.03.2002 */
	  if(AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i] == NULL)
	    AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i] = MY_ALLOC(axis_list_el,1);
	  /* E 12.03.2002 */
	  AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i]->the_bb = group_below[i];
	  AABB_TREE[hierheight][group_nr].aabb_zlist[group_below_nr+i]->val = all_bbs[group_below[i]]->z2;
	}
    }
  /* B 12.03.2002 */
  AABB_TREE[hierheight][group_nr].group_below_nr = group_below_nr;
  /* E 12.03.2002 */
  kcd_sort_aabb_tree(AABB_TREE[hierheight][group_nr].aabb_xlist, 2*group_below_nr);
  kcd_sort_aabb_tree(AABB_TREE[hierheight][group_nr].aabb_ylist, 2*group_below_nr);
  kcd_sort_aabb_tree(AABB_TREE[hierheight][group_nr].aabb_zlist, 2*group_below_nr);
}

/* ****************************************************************************************** *
 * Function :  generate_obb_tree_on_boxes()
 *             computes an OBB tree on the boxes belonging to the same "KCD object"
 *             the "KCD object" is a grouping at level 1 in the AABB_TREE
 * ARGS IN  :  hierheight             index of level in AABB_TREE
 *             group_nr               index on level  hierheight of a group of BBs stored 
 *                                    in AABB_TREE
 *             the_aabb_group         the array of elements in the group with indices  
 *                                    hierheight and group_nr
 *             nr_elements_in_group   number of elements in array  the_aabb_group
 * ****************************************************************************************** */
static void generate_obb_tree_on_boxes(int hierheight,int group_nr, int *the_aabb_group, int nr_elements_in_group)
{
  int i,j,nof_orig_obbs = nr_elements_in_group;
  kcd_bb_p *the_orig_obbs = NULL;
  kcd_vector3 *the_points = NULL;
  int nb_points;
  int **the_facet_vertices = NULL;
  double bx,by,bz;
  kcd_vector3 eigenv[3];
  kcd_vector3 center;

  the_orig_obbs = MY_ALLOC(kcd_bb_p,nof_orig_obbs);
  nb_points = 8*nof_orig_obbs;
  the_points = MY_ALLOC(kcd_vector3,nb_points);
  the_facet_vertices = MY_ALLOC(int *,nof_orig_obbs);
  for(i=0;i<nof_orig_obbs;i++)
    {
      the_facet_vertices[i] = MY_ALLOC(int,8);
      /* keep track of the original obbs */
      the_orig_obbs[i] = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb];
      /* get size of OBB */
      bx = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->d[0];
      by = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->d[1];
      bz = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->d[2];
      /* get orientation of OBB */
      kcd_vectCopy(all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->eigenv[0],eigenv[0]);
      kcd_vectCopy(all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->eigenv[1],eigenv[1]);
      kcd_vectCopy(all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->eigenv[2],eigenv[2]);
      /* get placement of OBB */
      center[0] = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->center[0];
      center[1] = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->center[1];
      center[2] = all_bbs[AABB_TREE[hierheight][the_aabb_group[i]].the_bb]->center[2];
      /* size * eigenv */
      eigenv[0][0] = bx * eigenv[0][0];
      eigenv[0][1] = bx * eigenv[0][1];
      eigenv[0][2] = bx * eigenv[0][2];
      eigenv[1][0] = by * eigenv[1][0];
      eigenv[1][1] = by * eigenv[1][1];
      eigenv[1][2] = by * eigenv[1][2];
      eigenv[2][0] = bz * eigenv[2][0];
      eigenv[2][1] = bz * eigenv[2][1];
      eigenv[2][2] = bz * eigenv[2][2];
      /* create points */
      /* point 0 */
      the_points[0+i*8][0] = center[0] - eigenv[0][0] - eigenv[1][0] - eigenv[2][0];
      the_points[0+i*8][1] = center[1] - eigenv[0][1] - eigenv[1][1] - eigenv[2][1];
      the_points[0+i*8][2] = center[2] - eigenv[0][2] - eigenv[1][2] - eigenv[2][2];
      /* point 1 */
      the_points[1+i*8][0] = center[0] + eigenv[0][0] - eigenv[1][0] - eigenv[2][0];
      the_points[1+i*8][1] = center[1] + eigenv[0][1] - eigenv[1][1] - eigenv[2][1];
      the_points[1+i*8][2] = center[2] + eigenv[0][2] - eigenv[1][2] - eigenv[2][2];
      /* point 2 */
      the_points[2+i*8][0] = center[0] + eigenv[0][0] - eigenv[1][0] + eigenv[2][0];
      the_points[2+i*8][1] = center[1] + eigenv[0][1] - eigenv[1][1] + eigenv[2][1];
      the_points[2+i*8][2] = center[2] + eigenv[0][2] - eigenv[1][2] + eigenv[2][2];
      /* point 3 */
      the_points[3+i*8][0] = center[0] - eigenv[0][0] - eigenv[1][0] + eigenv[2][0];
      the_points[3+i*8][1] = center[1] - eigenv[0][1] - eigenv[1][1] + eigenv[2][1];
      the_points[3+i*8][2] = center[2] - eigenv[0][2] - eigenv[1][2] + eigenv[2][2];
      /* point 4 */
      the_points[4+i*8][0] = center[0] - eigenv[0][0] + eigenv[1][0] - eigenv[2][0];
      the_points[4+i*8][1] = center[1] - eigenv[0][1] + eigenv[1][1] - eigenv[2][1];
      the_points[4+i*8][2] = center[2] - eigenv[0][2] + eigenv[1][2] - eigenv[2][2];
      /* point 5 */
      the_points[5+i*8][0] = center[0] + eigenv[0][0] + eigenv[1][0] - eigenv[2][0];
      the_points[5+i*8][1] = center[1] + eigenv[0][1] + eigenv[1][1] - eigenv[2][1];
      the_points[5+i*8][2] = center[2] + eigenv[0][2] + eigenv[1][2] - eigenv[2][2];
      /* point 6 */
      the_points[6+i*8][0] = center[0] + eigenv[0][0] + eigenv[1][0] + eigenv[2][0];
      the_points[6+i*8][1] = center[1] + eigenv[0][1] + eigenv[1][1] + eigenv[2][1];
      the_points[6+i*8][2] = center[2] + eigenv[0][2] + eigenv[1][2] + eigenv[2][2];
      /* point 7 */
      the_points[7+i*8][0] = center[0] - eigenv[0][0] + eigenv[1][0] + eigenv[2][0];
      the_points[7+i*8][1] = center[1] - eigenv[0][1] + eigenv[1][1] + eigenv[2][1];
      the_points[7+i*8][2] = center[2] - eigenv[0][2] + eigenv[1][2] + eigenv[2][2];

      for(j=0;j<8;j++)
	the_facet_vertices[i][j] = i*8+j+1;
    }
  /* call OBB-tree construction on these points */
  enclose_obbs_with_obb_tree(AABB_TREE[hierheight+1][group_nr].the_bb, nof_orig_obbs, the_orig_obbs, the_points, nb_points, the_facet_vertices);

  MY_FREE(the_points,kcd_vector3,nb_points);
  for(i=0;i<nof_orig_obbs;i++)
    {
      MY_FREE(the_facet_vertices[i],int,8);
    }
  MY_FREE(the_facet_vertices,int *,nof_orig_obbs);
  MY_FREE(the_orig_obbs,kcd_bb_p,nof_orig_obbs);
}

static void unset_aabb_to_leaf(int hierheight, int group_nr)
{
  int k,kids;

  /* set this one to FALSE */
  all_bbs[AABB_TREE[hierheight][group_nr].the_bb]->is_aabb_tree_leaf = FALSE;
  /* recursively set all kids to false */
  kids = AABB_TREE[hierheight][group_nr].group_below_nr;
  for(k=0;k<kids;k++)
    unset_aabb_to_leaf(hierheight-1,AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el[k]);
}

static void set_aabb_to_leaf(int hierheight, int group_nr)
{
  int k,kids;

  all_bbs[AABB_TREE[hierheight][group_nr].the_bb]->is_aabb_tree_leaf = TRUE;

  /* set kids to false */
  kids = AABB_TREE[hierheight][group_nr].group_below_nr;
  for(k=0;k<kids;k++)
    unset_aabb_to_leaf(hierheight-1,AABB_TREE[hierheight][group_nr].group_below_aabb_hier_el[k]);

  /* some flags, probably useless */
  all_bbs[AABB_TREE[hierheight][group_nr].the_bb]->current_status_aabb = UNCHANGED ;
  all_bbs[AABB_TREE[hierheight][group_nr].the_bb]->current_status_obb = UNCHANGED ;
}

/* ****************************************************************************************** *
 * Function : create_aabb_hierarchy()
 *            create the aabb tree on the collection of static objects 
 * ****************************************************************************************** */
void create_aabb_hierarchy()
{
  int first_new_bb=0;  //removed warning of mightbe uninitialized
  int i,j,hierheight/*,jj*/;
  double dist = 0.0;
  int *closest=NULL;
  int *first_overlap=NULL;
  int nr_p3d_objs;
  double *mindist=NULL;
  double global_mindist;
  int bb;
  int tot_gr_nr = 0, gr_nr, nofels;
  int **aabb_group = NULL;
  int *aabb_group_nofels = NULL;
  int *track_aabb_group = NULL;
  int group_bb_nr,pol_nr;
  int kcd_sml_prim_i, kcd_big_prim_i,ith_so;
  double x,y,z;
  int change;

  /* for each static primitive */
  nr_p3d_objs = kcd_get_nof_sos();
  pol_nr = 0;
  for(i=0;i<nr_p3d_objs;i++)
    {
      ith_so = ith_static_obj[i];

      kcd_sml_prim_i = kcd_o_prm_table[ith_so].sml_index;
      kcd_big_prim_i = kcd_o_prm_table[ith_so].big_index;
      for(j=kcd_sml_prim_i;j<=kcd_big_prim_i;j++)
	{
	  bb = kcd_obj_prim_table[j].kcd_prim_bb_id;
	  if(bb != -1)
	    {
	      pol_nr++;
	      insert_static_aabb(bb,0,j,NULL,0);
	    }
	}
    }

  if(pol_nr > 0)
    {
      hierheight = 0;
      global_mindist = -5.0; /* init as negative distance */
      while(LEQ(global_mindist,0.0))
	{
	  global_mindist = 5.0; /* init as strictly positive distance */
	  /* recursively add a layer to the hierarchy by the following operations */
	  /* determine neighbourness, for each of the static AABBs */
	  closest = MY_ALLOC(int,nr_aabb_hier_el[hierheight]);
	  first_overlap = MY_ALLOC(int,nr_aabb_hier_el[hierheight]);
	  mindist = MY_ALLOC(double,nr_aabb_hier_el[hierheight]);
	  track_aabb_group = MY_ALLOC(int,nr_aabb_hier_el[hierheight]);
	  for(i=0;i<nr_aabb_hier_el[hierheight];i++)
	    {
	      first_overlap[i] = i;
	      mindist[i] = 0.0;
	    }
	  do
	    {
	      change = FALSE;
	      for(i=0;i<nr_aabb_hier_el[hierheight];i++)
		{
		  /* initialisation: mindist = diagonal of workspace */
		  if(first_overlap[i]==i)
		    {
		      kcd_get_scene_size(&x,&y,&z);
		      mindist[i] = ( x*x + y*y + z*z );
		    }
		  
		  /* determine smallest distance or largest penetration 
		     and keep track of the other AABB with which it is close */
		  for(j=i+1;j<nr_aabb_hier_el[hierheight];j++)
		    if (first_overlap[i]!=first_overlap[j])
		      {
			dist = aabbs_square_distance(
						     AABB_TREE[hierheight][i].the_bb,
						     AABB_TREE[hierheight][j].the_bb); /* we just need the square distance */
			
			if((EQ(dist,0.0)) ) 
			  {
			    dist = - aabbs_square_penetration_measure_when_collision(
										     AABB_TREE[hierheight][i].the_bb,
										     AABB_TREE[hierheight][j].the_bb);
			    if((i<j)&&
			       ( ((hierheight == 0)&&(new_kcd_belong_to_same_object(i,j)))  || 
				 ((hierheight == 1)&&(kcd_empty_volume_small_after_union(hierheight,i,j)) ) || 
				 (hierheight > 1) 
				 )
			       )
			      {
				first_overlap[j]=MIN(first_overlap[j],first_overlap[i]);
				first_overlap[i]=first_overlap[j];
				change = TRUE;
			      }
			    /* else first_overlap[i] already set */
			  }
			if(dist < mindist[i])
			  {
			    mindist[i] = dist;
			    closest[i]=j;
			  }
			if(mindist[i] < global_mindist)
			  global_mindist = mindist[i];
		      }
		}
	    }while (change);
	  
	  for(i=0;i<nr_aabb_hier_el[hierheight];i++)
	    {
	      /* group AABBs by connected component 
		 in the graph of nearest AABBs */
	      if(hierheight == 0)
		{
		  if((LEQ(mindist[i],0.0))&&(first_overlap[i] < i))
		    {
		      /* look-up which grouping contains first_overlap[i] */
		      gr_nr = track_aabb_group[first_overlap[i]];
		      /* put i in grouping of first_overlap[gr_nr] */
		      aabb_group[gr_nr] = MY_REALLOC(aabb_group[gr_nr],int,aabb_group_nofels[gr_nr],aabb_group_nofels[gr_nr]+1);
		      nofels = ++aabb_group_nofels[gr_nr];
		      aabb_group[gr_nr][nofels-1] = i;
		      /* keep track of the grouping in which one puts i */
		      track_aabb_group[i] = gr_nr;
		    }
		  else
		    {
		      aabb_group = MY_REALLOC(aabb_group,intp,tot_gr_nr,tot_gr_nr+1);
		      aabb_group_nofels = MY_REALLOC(aabb_group_nofels,int,tot_gr_nr,tot_gr_nr+1);
		      tot_gr_nr++;
		      /* new grouping */
		      aabb_group[tot_gr_nr-1] = MY_ALLOC(int,1);
		      aabb_group[tot_gr_nr-1][0] = i;
		      aabb_group_nofels[tot_gr_nr-1] = 1;
		      /* keep track of the grouping in which one puts i */
		      track_aabb_group[i] = tot_gr_nr-1;
		    }
		}
	      else
		{
		  if((LEQ(mindist[i],0.0))&&(first_overlap[i] < i))
		    {
		      /* look-up which grouping contains first_overlap[i] */
		      gr_nr = track_aabb_group[first_overlap[i]];
		      /* put i in grouping of first_overlap[gr_nr] */
		      aabb_group[gr_nr] = MY_REALLOC(aabb_group[gr_nr],int,aabb_group_nofels[gr_nr],aabb_group_nofels[gr_nr]+1);
		      nofels = ++aabb_group_nofels[gr_nr];
		      aabb_group[gr_nr][nofels-1] = i;
		      /* keep track of the grouping in which one puts i */
		      track_aabb_group[i] = gr_nr;
		    }
		  else
		    {
		      aabb_group = MY_REALLOC(aabb_group,intp,tot_gr_nr,tot_gr_nr+1);
		      aabb_group_nofels = MY_REALLOC(aabb_group_nofels,int,tot_gr_nr,tot_gr_nr+1);
		      tot_gr_nr++;
		      /* new grouping */
		      aabb_group[tot_gr_nr-1] = MY_ALLOC(int,1);
		      aabb_group[tot_gr_nr-1][0] = i;
		      aabb_group_nofels[tot_gr_nr-1] = 1;
		      /* keep track of the grouping in which one puts i */
		      track_aabb_group[i] = tot_gr_nr-1;
		    }
		}
	    }
	  
	  if(tot_gr_nr < nr_aabb_hier_el[hierheight])
	    {
	      /* we really grouped something */
	      
	      /* put an AABB around each group */
	      /* and make next hierarchy level */
	      /* resize_hash_table_on_bbs(tot_gr_nr); */
	      for(gr_nr=0;gr_nr<tot_gr_nr;gr_nr++)
		{
		  group_bb_nr = kcd_determine_aabb_around_aabbs(aabb_group[gr_nr],aabb_group_nofels[gr_nr],hierheight);
		  if(gr_nr==0)
       		      first_new_bb = group_bb_nr; 	
		}
	      /* add new bb to list of bbs */
	      make_hash_table_on_bbs();
	      for(gr_nr=0;gr_nr<tot_gr_nr;gr_nr++)
		{		 
		  group_bb_nr = first_new_bb + gr_nr;
		  /* put bb in next hierarchy level */
		  insert_static_aabb(group_bb_nr-1,hierheight+1,gr_nr,aabb_group[gr_nr],aabb_group_nofels[gr_nr]);
		}
	      /* if we added a level on top of hierheight == 0, we will now consider the level
		 hierheight == 1 as the lowest AABB level and construct a OBB-Tree on the collection
		 of children of each of the boxes on level hierheight == 1 */
	      if(hierheight == 0) 
		{
		  /* tell the user */
		  PrintInfo(("KCD: building OBB-tree on static objects...\n"));
		  for(gr_nr=0;gr_nr<tot_gr_nr;gr_nr++)
		    {
		      if(aabb_group_nofels[gr_nr] > 1)
			{
			  /* construct a OBB-Tree for the (gr_nr)-th AABB on hierheight == 1 */ 
			  generate_obb_tree_on_boxes(hierheight,gr_nr,aabb_group[gr_nr], aabb_group_nofels[gr_nr]);
			  /* make sure aabb-leaf  now is  (gr_nr)-th AABB on hierheight == 1 */
			  set_aabb_to_leaf(hierheight,gr_nr); /* BUG should be hierheight+1 but exact distance does'nt work */
			}
		    }
		  make_hash_table_on_bbs();
		  /* tell the user */
		  PrintInfo(("KCD: building OBB-tree on static objects done.\n"));
		}
	    }
	  else
	    {
	      /* we didn't group anything, so we stop the construction of the hierarchy */
	      global_mindist = 5555.5555;
	      /* put a big box around all boxes in previous level */
	      aabb_group = MY_REALLOC(aabb_group,intp,tot_gr_nr,tot_gr_nr+1);
	      aabb_group_nofels = MY_REALLOC(aabb_group_nofels,int,tot_gr_nr,tot_gr_nr+1);
	      /* new grouping */
	      aabb_group[tot_gr_nr] = MY_ALLOC(int,nr_aabb_hier_el[hierheight]);
	      aabb_group_nofels[tot_gr_nr] = nr_aabb_hier_el[hierheight];
	      for(i=0;i<nr_aabb_hier_el[hierheight];i++)
		{
		  aabb_group[tot_gr_nr][i] = i;
		}
	      group_bb_nr = kcd_determine_aabb_around_aabbs(aabb_group[tot_gr_nr],aabb_group_nofels[tot_gr_nr],hierheight);	
	      make_hash_table_on_bbs();
	      insert_static_aabb(group_bb_nr-1,hierheight+1,0,aabb_group[tot_gr_nr],aabb_group_nofels[tot_gr_nr]);
	      tot_gr_nr++; /* in order to clean up */
	    }
	  if (tot_gr_nr == 1)
	    {
	      /* we grouped everything in this group: stop the construction of the hierarchy */
	      global_mindist = 5555.5555;
	    }
	  
	  if(hierheight >= 25)
	    {
	      /* we don't want more levels than this */
	      global_mindist = 5555.5555;
	      /* put a big box around all boxes in previous level */
	      aabb_group = MY_REALLOC(aabb_group,intp,tot_gr_nr,tot_gr_nr+1);
	      aabb_group_nofels = MY_REALLOC(aabb_group_nofels,int,tot_gr_nr,tot_gr_nr+1);
	      /* new grouping */
	      aabb_group[tot_gr_nr] = MY_ALLOC(int,nr_aabb_hier_el[hierheight]);
	      aabb_group_nofels[tot_gr_nr] = nr_aabb_hier_el[hierheight];
	      for(i=0;i<nr_aabb_hier_el[hierheight];i++)
		{
		  aabb_group[tot_gr_nr][i] = i;
		}
	      group_bb_nr = kcd_determine_aabb_around_aabbs(aabb_group[tot_gr_nr],nr_aabb_hier_el[hierheight],hierheight);

	      make_hash_table_on_bbs();
	      insert_static_aabb(group_bb_nr-1,hierheight+1,0,aabb_group[tot_gr_nr],aabb_group_nofels[tot_gr_nr]);
	      tot_gr_nr++; /* in order to clean up */
	    }
	  hierheight++;
	  
	  /* clean up local stuff */
	  for(i=0;i<tot_gr_nr;i++)
	    {
	      MY_FREE(aabb_group[i],intp,aabb_group_nofels[i]);
	      aabb_group[i]=NULL;
	    }
	  MY_FREE(aabb_group_nofels,int,tot_gr_nr);
	  aabb_group_nofels=NULL;
	  MY_FREE(aabb_group,intp,tot_gr_nr);
	  aabb_group=NULL;
	  tot_gr_nr = 0;
	  MY_FREE(closest,int,nr_aabb_hier_el[hierheight-1]);
	  closest=NULL;
	  MY_FREE(first_overlap,int,nr_aabb_hier_el[hierheight-1]);
	  first_overlap=NULL;
	  MY_FREE(mindist,double,nr_aabb_hier_el[hierheight-1]);
	  mindist=NULL;
	  MY_FREE(track_aabb_group,int,nr_aabb_hier_el[hierheight-1]);
	  track_aabb_group=NULL;
	}
    }
  else
    {
      /* there is no obstacle */
      hierheight = 0;
      make_hash_table_on_bbs();
      insert_static_aabb(0,hierheight,0,NULL,0);
    }
}
  

/* FOR MOVING OBJECTS (links, free-flyers */

int kcd_study_candidate(int hier_level,int el_at_level,int mobj_id, kcd_bb *robot_bb, int *tot_nof_res,
			kcd_matrix4 *mat2, int with_report, double *minimal_distance_estimate, int *nearest_obstacle, double the_tolerance)
{
  int rob_id,link_id=0;
  int nof_kids, nof_res = 0, all_overlap = TRUE;
  int this_kid_overlaps = TRUE; //okid_in_collision;
  int next_el_at_level,next_kid,candidate_nearest_obstacle;
  double  a_distance, smallest_dist_at_this_level = P3D_HUGE;
#ifndef USE_P3D_BODY_AABB
  double aabb_d[3];
  kcd_matrix4 mat,invM2;
#endif
  /* B Kineo Carl 27.02.2002 */
  int ext_obj_id;
  /* E Kineo Carl 27.02.2002 */

  /* debug nearest obst  de Pepijn  august 2001*/ 
  candidate_nearest_obstacle = AABB_TREE[hier_level][el_at_level].the_bb;

  rob_id = kcd_mo_bb_table[mobj_id].grp_id;

  /* test this bbox */
#ifdef USE_P3D_BODY_AABB
  if(with_report != DISTANCE_ESTIMATE)
    {
      all_overlap = aabbs_overlap(robot_bb,all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]);
    }
  else
    {     
      smallest_dist_at_this_level = 
	aabbs_square_distance(kcd_mo_bb_table[mobj_id].bb_id,AABB_TREE[hier_level][el_at_level].the_bb);
      all_overlap = LEQ(smallest_dist_at_this_level,the_tolerance); /* security tolerance distance */
    }
#else
  invM2[0][0] = 1.0;
  invM2[0][1] = 0.0;
  invM2[0][2] = 0.0;
  invM2[1][0] = 0.0;
  invM2[1][1] = 1.0;
  invM2[1][2] = 0.0;
  invM2[2][0] = 0.0;
  invM2[2][1] = 0.0;
  invM2[2][2] = 1.0;
  invM2[3][0] = 0.0;
  invM2[3][1] = 0.0;
  invM2[3][2] = 0.0;
  invM2[3][3] = 1.0;

  invM2[0][3] = -(all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->x2 + all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->x1)/2.0;
  invM2[1][3] = -(all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->y2 + all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->y1)/2.0;
  invM2[2][3] = -(all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->z2 + all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->z1)/2.0;
  aabb_d[0] = (all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->x2 - all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->x1)/2.0;
  aabb_d[1] = (all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->y2 - all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->y1)/2.0;
  aabb_d[2] = (all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->z2 - all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->z1)/2.0;
  kcd_mat4Mult(invM2,*mat2,mat);   
  /* call OBB of link vs. AABB in hierarchy: use OBB vs. OBB test */
  all_overlap = !(obb_disjoint(mat,aabb_d,robot_bb->d));
#endif

  if((all_overlap) && !(all_bbs[AABB_TREE[hier_level][el_at_level].the_bb]->is_aabb_tree_leaf))
    {
      /* box collides and has some kids */
      /* PrintInfo(("aabbs_overlap() returned TRUE (not aabb-tree-leaf) \n")); */
      nof_kids = AABB_TREE[hier_level][el_at_level].group_below_nr;
      if(nof_kids > 0)
	{
	  /* start studying the kids */
	  next_kid = 0;
	  while((all_overlap)&&(next_kid < nof_kids))
	    {
	      /* B modif. Kineo Carl 27.02.2002 */
	      next_el_at_level = AABB_TREE[hier_level][el_at_level].group_below_aabb_hier_el[next_kid];
	      ext_obj_id = all_bbs[AABB_TREE[hier_level-1][next_el_at_level].the_bb]->ext_obj_id;
	      /* printf("loop 1:  ext_obj_id = %i\n",ext_obj_id); */
	      if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE)
		{
		  a_distance = P3D_HUGE;
		  all_overlap = all_overlap && 
		    kcd_study_candidate(hier_level-1, next_el_at_level,mobj_id, robot_bb,
					tot_nof_res, mat2,with_report, &a_distance, &candidate_nearest_obstacle, the_tolerance) ;
		}
	      else
		{
		  all_overlap = FALSE;
		}
	      /* E modif. Kineo Carl 27.02.2002 - intro. du if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE) + else */
	      next_kid++;
	    }


	  if(all_overlap)
	    {
	      /* all kids overlap, we just keep parent ! */
	      nof_interm_res[hier_level-1] -= nof_kids;

	      interm_res[hier_level][nof_interm_res[hier_level]].robot_id = rob_id;
	      interm_res[hier_level][nof_interm_res[hier_level]].link_id = link_id;
	      interm_res[hier_level][nof_interm_res[hier_level]].place_link = mobj_id;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.x1 = robot_bb->x1;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.x2 = robot_bb->x2;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.y1 = robot_bb->y1;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.y2 = robot_bb->y2;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.z1 = robot_bb->z1;
	      interm_res[hier_level][nof_interm_res[hier_level]].robot_box.z2 = robot_bb->z2;
	      interm_res[hier_level][nof_interm_res[hier_level]].static_object_bb_id = AABB_TREE[hier_level][el_at_level].the_bb;
	      interm_res[hier_level][nof_interm_res[hier_level]].hier_level = hier_level;
	      interm_res[hier_level][nof_interm_res[hier_level]].bb_id_at_level = el_at_level;	 
	      nof_interm_res[hier_level] += 1;
	      nof_res = 1;
	      if(with_report == DISTANCE_ESTIMATE)
		{
		  *nearest_obstacle = AABB_TREE[hier_level][el_at_level].the_bb;   
// 		  PrintInfo((" nearest %d \n",  *nearest_obstacle));
		}
	    }
	  else
	    {
	      /* last checked kid didn't intersect, study all following kids */
	      if(with_report == DISTANCE_ESTIMATE)
		{
		  if(GNEQ(*minimal_distance_estimate,a_distance))
		    {
		      *minimal_distance_estimate = a_distance; 
//		      PrintInfo((" candidate %d \n",  candidate_nearest_obstacle));
		      *nearest_obstacle = candidate_nearest_obstacle;
		    }
		}
	      while(next_kid < nof_kids)
		{
		  /* B modif. Kineo Carl 27.02.2002 */
		  next_el_at_level = AABB_TREE[hier_level][el_at_level].group_below_aabb_hier_el[next_kid];
		  ext_obj_id = all_bbs[AABB_TREE[hier_level-1][next_el_at_level].the_bb]->ext_obj_id;
		  /* printf("loop 2: ext_obj_id = %i\n",ext_obj_id); */
		  if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE)
		    {
		      a_distance = P3D_HUGE;

		      this_kid_overlaps = 
			kcd_study_candidate(hier_level-1, next_el_at_level,mobj_id, robot_bb,
					    tot_nof_res, mat2, with_report,&a_distance, &candidate_nearest_obstacle, the_tolerance) ;
		      if(with_report == DISTANCE_ESTIMATE)
			{
			  if(GNEQ(*minimal_distance_estimate,a_distance))
			    {
			      *minimal_distance_estimate = a_distance;
			      *nearest_obstacle = candidate_nearest_obstacle;
			      //		  PrintInfo((" candidate %d \n",  candidate_nearest_obstacle));
			    }
			}
		    }
		  else
		    {
		      this_kid_overlaps = FALSE;
		    }
		  /* E modif. Kineo Carl 27.02.2002 - intro. du if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE) + else */
		  next_kid++;
		}
	    }
	}
    }
  else
    {   
      if(!all_overlap)
	{
	  /* box doesn't intersect (and we don't care about being it a leaf or not) */
	  /* ignore bbox in OBB test */
	  nof_res = 0;	  
	  /* this is a minimal distance candidate */
	  if(with_report == DISTANCE_ESTIMATE)
	    {
	      if(GNEQ(*minimal_distance_estimate,smallest_dist_at_this_level))
		{
		  *minimal_distance_estimate = smallest_dist_at_this_level;
		  *nearest_obstacle = candidate_nearest_obstacle; 
		}
	    }
	}
      else
	{
	  /* box is leaf and intersects */
	  if(with_report == DISTANCE_ESTIMATE)
	    {
	      /* *minimal_distance_estimate = 0.0;  */ /* 23032001 */
	      *nearest_obstacle = candidate_nearest_obstacle;
	    }
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_id = rob_id;
	  interm_res[hier_level][nof_interm_res[hier_level]].link_id = link_id;
	  interm_res[hier_level][nof_interm_res[hier_level]].place_link = mobj_id;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.x1 = robot_bb->x1;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.x2 = robot_bb->x2;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.y1 = robot_bb->y1;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.y2 = robot_bb->y2;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.z1 = robot_bb->z1;
	  interm_res[hier_level][nof_interm_res[hier_level]].robot_box.z2 = robot_bb->z2;
	  interm_res[hier_level][nof_interm_res[hier_level]].static_object_bb_id = AABB_TREE[hier_level][el_at_level].the_bb;
	  interm_res[hier_level][nof_interm_res[hier_level]].hier_level = hier_level;
	  interm_res[hier_level][nof_interm_res[hier_level]].bb_id_at_level = el_at_level;	  
	  nof_interm_res[hier_level] += 1;
	  nof_res = 1;
	}
    }
  /* return result */
  *tot_nof_res += nof_res;

  return all_overlap;
}

void kcd_init_robot_vs_static_aabbs(int mobj_id, double x1,double x2,double y1,double y2,
				    double z1,double z2, int *tot_nof_res)
{
  int rob_id,link_id=0;
  int i,j,count_res,place_link=mobj_id;
  double aabb_tolerance,the_tolerance = 0.0;
 //  kcd_matrix4 *jnt_pos;

#ifdef USE_P3D_BODY_AABB
  kcd_bb robot_bb;
#else
  kcd_bb_p robot_bb_p;
  kcd_matrix4 M1,mat2;
#endif

  rob_id = kcd_mo_bb_table[mobj_id].grp_id;

  /* 1212 init */
  *tot_nof_res = 0;
  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      nof_interm_res[i] = 0;
    }  

#ifdef USE_P3D_BODY_AABB
  robot_bb.x1 = x1;
  robot_bb.x2 = x2;
  robot_bb.y1 = y1;
  robot_bb.y2 = y2;
  robot_bb.z1 = z1;
  robot_bb.z2 = z2;
#else
  /* get the OBB around the link */
  robot_bb_p = all_bbs[kcd_mo_bb_table[mobj_id].bb_id];
  /* robot_bb_p = all_bbs[kcd_body_bb_table[rob_id][link_id].bb_id]; */


  robot_bb_p->x1 = x1;
  robot_bb_p->x2 = x2;
  robot_bb_p->y1 = y1;
  robot_bb_p->y2 = y2;
  robot_bb_p->z1 = z1;
  robot_bb_p->z2 = z2;

  M1[0][0] = robot_bb_p->eigenv[0][0];
  M1[1][0] = robot_bb_p->eigenv[0][1];
  M1[2][0] = robot_bb_p->eigenv[0][2];
  M1[0][3] = robot_bb_p->center[0];
  M1[0][1] = robot_bb_p->eigenv[1][0];
  M1[1][1] = robot_bb_p->eigenv[1][1];
  M1[2][1] = robot_bb_p->eigenv[1][2];
  M1[1][3] = robot_bb_p->center[1];
  M1[0][2] = robot_bb_p->eigenv[2][0];
  M1[1][2] = robot_bb_p->eigenv[2][1];
  M1[2][2] = robot_bb_p->eigenv[2][2];
  M1[2][3] = robot_bb_p->center[2];
  M1[3][0] = 0.0;
  M1[3][1] = 0.0;
  M1[3][2] = 0.0;
  M1[3][3] = 1.0;
  
  kcd_get_obj_abs_pos(mobj_id,&jnt_pos);
  kcd_mat4Mult(*jnt_pos,M1,mat2);
#endif

  kcd_get_tolerance(&the_tolerance);
  aabb_tolerance = the_tolerance * the_tolerance;

#ifdef USE_P3D_BODY_AABB
  if(kcd_study_candidate(nr_kcd_aabb_list-1,0,mobj_id,&robot_bb,tot_nof_res,NULL,FALSE,NULL,NULL,aabb_tolerance))
#else
  if(kcd_study_candidate(nr_kcd_aabb_list-1,0,mobj_id,robot_bb_p,tot_nof_res,&mat2,FALSE,NULL,NULL,aabb_tolerance))
#endif
    {
      /* replace interm_res[] by the root */
      robots_vs_static_aabbs_in_col[place_link][0].robot_id = rob_id;
      robots_vs_static_aabbs_in_col[place_link][0].link_id = link_id;
      robots_vs_static_aabbs_in_col[place_link][0].place_link = place_link;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.x1 = x1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.x2 = x2;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.y1 = y1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.y2 = y2;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.z1 = z1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.z2 = z2;
      robots_vs_static_aabbs_in_col[place_link][0].static_object_bb_id = AABB_TREE[nr_kcd_aabb_list-1][0].the_bb;
      robots_vs_static_aabbs_in_col[place_link][0].hier_level = nr_kcd_aabb_list-1;
      robots_vs_static_aabbs_in_col[place_link][0].bb_id_at_level = 0;
      robots_vs_static_aabbs_in_col[place_link][0].some_obb_collides = FALSE; /* default value */
      *tot_nof_res = 1;
    }
  else
    {
      /* VISIT all interm_res, put them in res */
      count_res = 0;
      for( i=0; i<nr_kcd_aabb_list; i++ )
	{
	  for( j=0; j<nof_interm_res[i]; j++ )
 	    {
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_id = interm_res[i][j].robot_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].link_id = interm_res[i][j].link_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].place_link = interm_res[i][j].place_link;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.x1 = interm_res[i][j].robot_box.x1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.x2 = interm_res[i][j].robot_box.x2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.y1 = interm_res[i][j].robot_box.y1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.y2 = interm_res[i][j].robot_box.y2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.z1 = interm_res[i][j].robot_box.z1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.z2 = interm_res[i][j].robot_box.z2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].static_object_bb_id = interm_res[i][j].static_object_bb_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].hier_level = interm_res[i][j].hier_level;
	      robots_vs_static_aabbs_in_col[place_link][count_res].bb_id_at_level = interm_res[i][j].bb_id_at_level;
	      robots_vs_static_aabbs_in_col[place_link][count_res].some_obb_collides = FALSE; /* default value */
	      count_res++;
	    }
	}
      *tot_nof_res = count_res;
    }
}

int kcd_update_robot_vs_static_aabbs(int mobj_id,double x1,double x2,double y1,double y2,
				      double z1,double z2, int *tot_nof_res, int with_report, double *min_dist)
{
  int collision_found; /* 23032001 */
  int nearest_obstacle = -3,i,j,count_res,place_link=mobj_id,rob_id,link_id=0;
  /*  int nof_res = 0; */
  double aabb_tolerance,the_tolerance = 0.0;
  /* kcd_bb robot_bb; */
  kcd_bb_p robot_bb_p;
  double resulting_minimal_distance_estimate = P3D_HUGE;
  double *aabb_minimal_distance_estimate = NULL;
#ifndef USE_P3D_BODY_AABB
  kcd_matrix4 M1,mat2,*jnt_pos;
#endif

  rob_id = kcd_mo_bb_table[mobj_id].grp_id;
  if(with_report)
    aabb_minimal_distance_estimate = &resulting_minimal_distance_estimate;


  /* get the OBB around the link */
  robot_bb_p = all_bbs[kcd_mo_bb_table[mobj_id].bb_id];

  robot_bb_p->x1 = x1;
  robot_bb_p->x2 = x2;
  robot_bb_p->y1 = y1;
  robot_bb_p->y2 = y2;
  robot_bb_p->z1 = z1;
  robot_bb_p->z2 = z2;
#ifndef USE_P3D_BODY_AABB
  M1[0][0] = robot_bb_p->eigenv[0][0];
  M1[1][0] = robot_bb_p->eigenv[0][1];
  M1[2][0] = robot_bb_p->eigenv[0][2];
  M1[0][3] = robot_bb_p->center[0];
  M1[0][1] = robot_bb_p->eigenv[1][0];
  M1[1][1] = robot_bb_p->eigenv[1][1];
  M1[2][1] = robot_bb_p->eigenv[1][2];
  M1[1][3] = robot_bb_p->center[1];
  M1[0][2] = robot_bb_p->eigenv[2][0];
  M1[1][2] = robot_bb_p->eigenv[2][1];
  M1[2][2] = robot_bb_p->eigenv[2][2];
  M1[2][3] = robot_bb_p->center[2];
  M1[3][0] = 0.0;
  M1[3][1] = 0.0;
  M1[3][2] = 0.0;
  M1[3][3] = 1.0;
  
  kcd_get_obj_abs_pos(mobj_id,&jnt_pos);
  kcd_mat4Mult(*jnt_pos,M1,mat2);
#endif

  *tot_nof_res = 0;
  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      nof_interm_res[i] = 0;
    }  
  kcd_get_tolerance(&the_tolerance);
  aabb_tolerance = the_tolerance * the_tolerance;

#ifdef USE_P3D_BODY_AABB
  if(kcd_study_candidate(nr_kcd_aabb_list-1,0,mobj_id,robot_bb_p,tot_nof_res,NULL,with_report,aabb_minimal_distance_estimate,&nearest_obstacle,aabb_tolerance))
#else
  if(kcd_study_candidate(nr_kcd_aabb_list-1,0,mobj_id,robot_bb_p,tot_nof_res,&mat2,with_report,aabb_minimal_distance_estimate,&nearest_obstacle,aabb_tolerance))
#endif
    {
      collision_found = FALSE; /* 23032001 */

      robots_vs_static_aabbs_in_col[place_link][0].robot_id = rob_id;
      robots_vs_static_aabbs_in_col[place_link][0].link_id = link_id;
      robots_vs_static_aabbs_in_col[place_link][0].place_link = place_link;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.x1 = x1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.x2 = x2;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.y1 = y1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.y2 = y2;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.z1 = z1;
      robots_vs_static_aabbs_in_col[place_link][0].robot_box.z2 = z2;
      robots_vs_static_aabbs_in_col[place_link][0].static_object_bb_id = AABB_TREE[nr_kcd_aabb_list-1][0].the_bb;
      robots_vs_static_aabbs_in_col[place_link][0].hier_level = nr_kcd_aabb_list-1;
      robots_vs_static_aabbs_in_col[place_link][0].bb_id_at_level = 0;
      robots_vs_static_aabbs_in_col[place_link][0].some_obb_collides = FALSE; /* default value */
      *tot_nof_res = 1;
//      PrintInfo((" !!!!!!!"));
      /* set minimal distance to 0.0 */
      if(with_report)
	{
	  /* *aabb_minimal_distance_estimate = 0.0;  */ /* 23032001 */
	  kcd_modify_nearest_obstacle(rob_id,nearest_obstacle);
	//  PrintInfo(("** we modify, %d\n",nearest_obstacle ));
	}
    }
  else
    {
      collision_found = TRUE; /* 23032001 */

      /* VISIT all interm_res, put them in res */
      count_res = 0;
      for(i=0;i<nr_kcd_aabb_list;i++)
	{
	  for(j=0;j<nof_interm_res[i];j++)
 	    {
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_id     = interm_res[i][j].robot_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].link_id      = interm_res[i][j].link_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].place_link   = interm_res[i][j].place_link;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.x1 = interm_res[i][j].robot_box.x1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.x2 = interm_res[i][j].robot_box.x2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.y1 = interm_res[i][j].robot_box.y1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.y2 = interm_res[i][j].robot_box.y2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.z1 = interm_res[i][j].robot_box.z1;
	      robots_vs_static_aabbs_in_col[place_link][count_res].robot_box.z2 = interm_res[i][j].robot_box.z2;
	      robots_vs_static_aabbs_in_col[place_link][count_res].static_object_bb_id = interm_res[i][j].static_object_bb_id;
	      robots_vs_static_aabbs_in_col[place_link][count_res].hier_level = interm_res[i][j].hier_level;
	      robots_vs_static_aabbs_in_col[place_link][count_res].bb_id_at_level = interm_res[i][j].bb_id_at_level;
	      robots_vs_static_aabbs_in_col[place_link][count_res].some_obb_collides = FALSE; /* default value */
	      count_res++;
	    }		
	}
      *tot_nof_res = count_res;
//      PrintInfo((" !!!!!!!"));
      if(with_report)
	{
	  kcd_modify_nearest_obstacle(rob_id,nearest_obstacle);
	//  PrintInfo(("******* we modify, %d\n",nearest_obstacle ));
	}
     }
  /* aabbs_square_distance returns square of the distance, this function returns a distance estimate 
     between the given link and the nearest AABB */
  *min_dist = sqrt(resulting_minimal_distance_estimate); /* 23032001 */

  return collision_found; /* 23032001 */
}

void kcd_init_movable_stuff_robot(int i)
{
  double x1,x2,y1,y2,z1,z2;
  int bodynr,nof_bodies=0;
/*  int o_id;*/
  int kcd_ext_o,mobj_id,kcd_bb;

  nof_bodies = kcd_get_nof_grp_elts(i);
  for(bodynr=0;bodynr<nof_bodies;bodynr++)
    {
      mobj_id = kcd_get_body_bb_table(i, bodynr).kcd_mo_id;
      nof_robots_vs_static_aabbs[mobj_id] = 0;
      kcd_ext_o = kcd_mo_bb_table[mobj_id].ext_o_id;
      kcd_bb = kcd_mo_bb_table[mobj_id].bb_id;

      /* look for AABB around the kcd_ext_o-th movable object given by the user */
      kcd_get_mo_aabb(kcd_ext_o,kcd_bb,&x1,&x2,&y1,&y2,&z1,&z2);
      kcd_init_robot_vs_static_aabbs(mobj_id,x1,x2,y1,y2,z1,z2,&(nof_robots_vs_static_aabbs[mobj_id])); 
    }
}

/* ****************************************************************************************** *
 * Function: check_robot_vs_obbs_of_this_aabb() 
 * RETURNS : TRUE if no collision is found between 1. the kids (boxes, primitives,
 * or facets) of the AABB-leafs below the given AABB (a_bb) in the AABB-hierarchy and 2. 
 * the AABB of the link (a[4]) and the boxes, primitives and facets of the link, FALSE otherwise 
 * ****************************************************************************************** */
int check_robot_vs_obbs_of_this_aabb(int mobj_id, double a[4], double center_a[3], kcd_bb *a_bb, 
				     int with_report, double *minimal_distance_estimate, int *nearest_obstacle)
{
  kcd_matrix4 M1,invM2; /*M2,Place;*/
  double the_distance_estimate = P3D_HUGE;
  int i,obb_collision,kids_place_at_lower_level,aabb_tree_hier_level,
    aabb_tree_place_at_level,nof_kid_aabbs,no_clash_found = TRUE,the_nearest_obstacle;

//  PrintInfo(("we check robot ... "));
  if( a_bb->is_aabb_tree_leaf )
    {
      /* we only check each of the OBB(-trees) of the bodies if the AABB of the body collides */
      M1[0][0] = a_bb->eigenv[0][0];
      M1[1][0] = a_bb->eigenv[0][1];
      M1[2][0] = a_bb->eigenv[0][2];
      M1[0][3] = a_bb->center[0];
      M1[0][1] = a_bb->eigenv[1][0];
      M1[1][1] = a_bb->eigenv[1][1];
      M1[2][1] = a_bb->eigenv[1][2];
      M1[1][3] = a_bb->center[1];
      M1[0][2] = a_bb->eigenv[2][0];
      M1[1][2] = a_bb->eigenv[2][1];
      M1[2][2] = a_bb->eigenv[2][2];
      M1[2][3] = a_bb->center[2];
      M1[3][0] = 0.0;
      M1[3][1] = 0.0;
      M1[3][2] = 0.0;
      M1[3][3] = 1.0;
      
      invM2[0][0] = 1.0;
      invM2[0][1] = 0.0;
      invM2[0][2] = 0.0;
      invM2[1][0] = 0.0;
      invM2[1][1] = 1.0;
      invM2[1][2] = 0.0;
      invM2[2][0] = 0.0;
      invM2[2][1] = 0.0;
      invM2[2][2] = 1.0;
      invM2[3][0] = 0.0;
      invM2[3][1] = 0.0;
      invM2[3][2] = 0.0;
      invM2[3][3] = 1.0;

      invM2[0][3] = -center_a[0];
      invM2[1][3] = -center_a[1];
      invM2[2][3] = -center_a[2];
      if( (with_report == DISTANCE_ESTIMATE) || (with_report == DISTANCE_EXACT) )
	{
	 // PrintInfo(("we 're ...,"));
	  obb_collision = obb_disjoint_trees(mobj_id,M1,invM2,a,a_bb,with_report,&the_distance_estimate,&the_nearest_obstacle);
//	  PrintInfo(("tested obb_collision %d ,",obb_collision));
	}
      else
	{
	  obb_collision = obb_disjoint_trees(mobj_id,M1,invM2,a,a_bb,with_report,NULL,NULL);
	}
      if(!obb_collision) 
	{
	  /* obb-tree clashes with aabb */
	  no_clash_found = FALSE;
	  if(with_report == DISTANCE_ESTIMATE ||with_report == DISTANCE_EXACT)
	    {
	      *minimal_distance_estimate = 0.0;
	      *nearest_obstacle = the_nearest_obstacle;
	    }
	}
      else
	{
	  if(with_report == DISTANCE_ESTIMATE||with_report == DISTANCE_EXACT)
	    if(*minimal_distance_estimate > the_distance_estimate)
	      {
	//	PrintInfo(("we set in check\n"));
		*minimal_distance_estimate = the_distance_estimate;
		*nearest_obstacle = the_nearest_obstacle;
	      }
	}
    }
  else
    {
//      PrintInfo(("we see kids ???\n"));
      /* check the kids in the AABB-tree of the originally given AABB */
      aabb_tree_hier_level = a_bb->hier_level;
      aabb_tree_place_at_level = a_bb->place_at_level;
      nof_kid_aabbs = AABB_TREE[aabb_tree_hier_level][aabb_tree_place_at_level].group_below_nr;
     
      for(i=0;(i<nof_kid_aabbs)&&(no_clash_found);i++)
	{
	  kids_place_at_lower_level = AABB_TREE[aabb_tree_hier_level][aabb_tree_place_at_level].group_below_aabb_hier_el[i];
	  if(with_report == DISTANCE_ESTIMATE)
	    {
	      the_distance_estimate = P3D_HUGE;
	      no_clash_found = check_robot_vs_obbs_of_this_aabb(mobj_id, a, center_a,
								all_bbs[AABB_TREE[aabb_tree_hier_level-1][kids_place_at_lower_level].the_bb],with_report,&the_distance_estimate,&the_nearest_obstacle);
	      if(the_distance_estimate < *minimal_distance_estimate)
		{
		  *minimal_distance_estimate = the_distance_estimate;
		  *nearest_obstacle = the_nearest_obstacle;
		}
	    }
	  else
	    {
	      no_clash_found = check_robot_vs_obbs_of_this_aabb(mobj_id, a, center_a,
								all_bbs[AABB_TREE[aabb_tree_hier_level-1][kids_place_at_lower_level].the_bb],with_report,NULL,NULL);
	    }
	}
    }
  return no_clash_found;
}

int kcd_updated_robot_vs_static_obbs(int mobj_id, double xmin, double xmax, double ymin, 
				     double ymax, double zmin, double zmax, 
				     int with_report, double *minimal_distance_estimate, int *nearest_obstacle)
{
  int i,no_clash_found, place_link = mobj_id, the_nearest_obstacle;
  double a[4],center_a[3],the_distance_estimate=P3D_HUGE;

  /* AABB around movable object: */
  a[0] = xmax - xmin;
  a[1] = ymax - ymin;
  a[2] = zmax - zmin;
  /* a[3] contains largest value of a[0], a[1], a[2] */
  if(a[0] > a[1])
    if(a[0] > a[2])
      a[3] = a[0];
    else 
      a[3] = a[2];
  else
    if(a[1] > a[2])
      a[3] = a[1];
    else
      a[3] = a[2];

  center_a[0] = (xmax + xmin)/2.0;
  center_a[1] = (ymax + ymin)/2.0;
  center_a[2] = (zmax + zmin)/2.0;

  no_clash_found = TRUE;
  /* look at each of the AABBs in collision with this link */
  
  for(i=0; ( i<nof_robots_vs_static_aabbs[place_link] ) && ( no_clash_found );i++)
    {
      the_distance_estimate=P3D_HUGE;
      if(with_report == DISTANCE_ESTIMATE)
	{
	  no_clash_found = check_robot_vs_obbs_of_this_aabb(mobj_id,a,center_a,
		 all_bbs[robots_vs_static_aabbs_in_col[place_link][i].static_object_bb_id],with_report,&the_distance_estimate,&the_nearest_obstacle);
	  if(*minimal_distance_estimate > the_distance_estimate) 
	    {
	      *minimal_distance_estimate = the_distance_estimate;
	      *nearest_obstacle = the_nearest_obstacle;
	    }
	}
      else
	{
	  no_clash_found = check_robot_vs_obbs_of_this_aabb(mobj_id,a,center_a,
                 all_bbs[robots_vs_static_aabbs_in_col[place_link][i].static_object_bb_id],with_report,NULL,NULL);
	}
      if(no_clash_found == FALSE)
	{
	  robots_vs_static_aabbs_in_col[place_link][i].some_obb_collides = TRUE;
	}
    }
    
  return !no_clash_found;
}


/* ******************************************************************************* *
 * Function: kcd_mo_in_collision()
 * ARG IN  : mobj_id      mobj_id-th movable(!) object of the scene 
 * REMARKS : does not check for collision with other movable objects
 * ******************************************************************************* */
int kcd_mo_in_collision(int mobj_id,  int with_report, 
				   double *kcd_minimal_distance_estimate,int *nearest_obstacle)
{
  double x1=0.0,x2=0.0,y1=0.0,y2=0.0,z1=0.0,z2=0.0;
  double aabb_minimal_distance_estimate = P3D_HUGE,obb_minimal_distance_estimate = P3D_HUGE;
/*  int o_id;*/
  int kcd_ext_o,rob_id,kcd_bb;
  int j,obb_collide = FALSE, obb_nearest_obstacle;
/*  int cur_env_nr;*/
  int in_collision = FALSE;


  kcd_ext_o = kcd_mo_bb_table[mobj_id].ext_o_id;
  kcd_bb = kcd_mo_bb_table[mobj_id].bb_id;
  if(kcd_get_nof_prm_of_ext_o(kcd_ext_o) > 0)
    {
      /* look for the AABB around the kcd_ext_o-th movable object given by the user */
      //kcd_get_mo_aabb(kcd_ext_o,kcd_bb,&x1,&x2,&y1,&y2,&z1,&z2);

      /* change position of device, check AABB_TREE for collisions */
      /* FOR EACH OF THE LINKS ! */
/*       aabb_minimal_distance_estimate = */
/* 	kcd_update_robot_vs_static_aabbs(mobj_id,x1,x2,y1,y2,z1,z2,&(nof_robots_vs_static_aabbs[mobj_id]),with_report); */
      
      if (with_report == DISTANCE_EXACT)
	{ 
	  kcd_get_mo_aabb(kcd_ext_o,kcd_bb,&x1,&x2,&y1,&y2,&z1,&z2);
	  /* NOTE pepijn
	    we fill the values in the structures maybe it's better
	    to pass them as a parameter
	   */
	  all_bbs[kcd_bb]->x1 = x1;  all_bbs[kcd_bb]->x2 = x2;
	  all_bbs[kcd_bb]->y1 = y1;  all_bbs[kcd_bb]->y2 = y2;
	  all_bbs[kcd_bb]->z1 = z1;  all_bbs[kcd_bb]->z2 = z2;
	  obb_collide = kcd_dist_mo_in_collision(mobj_id,kcd_bb,kcd_minimal_distance_estimate);
	  if( obb_collide)
	    {
	      rob_id = kcd_mo_bb_table[mobj_id].grp_id;
	      if(rob_id != -1)
		kcd_add_report(rob_id,0);
	    }
//	  PrintInfo(("in aabb_tree %.0lf\n", *kcd_minimal_distance_estimate));
	  return (obb_collide);
	  
	}
      else
	{
	  kcd_get_mo_aabb(kcd_ext_o,kcd_bb,&x1,&x2,&y1,&y2,&z1,&z2);


	  in_collision = kcd_update_robot_vs_static_aabbs(mobj_id,x1,x2,y1,y2,z1,z2,&(nof_robots_vs_static_aabbs[mobj_id]),with_report, &aabb_minimal_distance_estimate); /* 23032001 */
	  
	  /* test all OBB(-tree)s below robots_vs_static_aabbs_in_col[][] */
	  /* if(aabb_minimal_distance_estimate > 0.0) */
	  if(!in_collision)
	    {
	      obb_minimal_distance_estimate = aabb_minimal_distance_estimate;
	    }
	
	  /* modif pepijn pour bug nearest obstacle */
	  rob_id = kcd_mo_bb_table[mobj_id].grp_id;
	  if(rob_id != -1)
	    obb_nearest_obstacle = kcd_get_nearest_obstacle_bb_id(rob_id);
	  else
	    obb_nearest_obstacle = 0; 
	  /* movable object on its own, no information on nearest obstacle */


//	  PrintInfo(("nearest after AABB :%d\n", obb_nearest_obstacle));

	  obb_collide = kcd_updated_robot_vs_static_obbs(mobj_id,x1,x2,y1,y2,z1,z2,
						     with_report,&obb_minimal_distance_estimate,&obb_nearest_obstacle);

	//  PrintInfo(("nearest after OBB :%d\n", obb_nearest_obstacle));
//	  PrintInfo(("obb_collides %d\n",obb_collide));
	  if(with_report)
	    {
	      if((obb_collide)&&(nof_robots_vs_static_aabbs[mobj_id]>0))
		{ 
		  for(j=0;j<nof_robots_vs_static_aabbs[mobj_id];j++)
		    {
		      if(robots_vs_static_aabbs_in_col[mobj_id][j].some_obb_collides)
			{
			  rob_id = kcd_mo_bb_table[mobj_id].grp_id;
			  if(rob_id != -1)
			    kcd_add_report(rob_id,robots_vs_static_aabbs_in_col[mobj_id][j].static_object_bb_id);
			}
		    }
		  if(with_report == DISTANCE_ESTIMATE)
		    {
		      *kcd_minimal_distance_estimate = 0.0;
		      *nearest_obstacle = obb_nearest_obstacle;
		    }
		  return TRUE;
		}
	      else
		{
		  /* kcd_set_report_to_zero(); */
		  if(with_report == DISTANCE_ESTIMATE)
		    {
		      if(obb_minimal_distance_estimate < aabb_minimal_distance_estimate)
			*kcd_minimal_distance_estimate = obb_minimal_distance_estimate;
		      else
			*kcd_minimal_distance_estimate = aabb_minimal_distance_estimate;
		      *nearest_obstacle = obb_nearest_obstacle;
		    }
		  return FALSE;
		}
	    }
	  else
	    {
	      return obb_collide;
	    }
	}
    }
  else
    {
      /* empty link */
      return FALSE;
    }
}

void kcd_delete_movable_stuff_robot(int i)
{
  if(interm_res)
    {
      for(i=0;i<nr_kcd_aabb_list;i++)
	{
	  if(interm_res[i])
	    {
	      MY_FREE(interm_res[i],robot_vs_static_aabb_collision,nr_aabb_hier_el[i]);
	      interm_res[i] = NULL;
	    }
	}
      MY_FREE(nof_interm_res,int,nr_kcd_aabb_list);
      MY_FREE(interm_res,robot_vs_static_aabb_collision_p,nr_kcd_aabb_list);
    }
  nof_interm_res = NULL;
  interm_res = NULL;
}


int kcd_robot_in_collision(int robot_id,int with_report, double *min_dist_estimate, int *nearest_obstacle)
{
  int i,nr_robots,collides = FALSE;

  /* self-collision */
  collides = kcd_robot_collides_itself(robot_id,with_report,min_dist_estimate, nearest_obstacle );
  /* collision with other robots */
  nr_robots = kcd_get_nof_grps();
  for(i=0;(i<nr_robots)&&(!collides);i++)
    {
      if(i!=robot_id)
	{
	  collides = kcd_robot_collides_robot(i,robot_id,with_report);
	}
    }
  /* collision with static objects */
  if(!collides)
    collides = kcd_robot_collides(robot_id,with_report,min_dist_estimate,nearest_obstacle);
  return collides;
}



void kcd_init_movable_stuff()
{
  int i,j,place_link,cand_max,max_nr=1;
  int nr_robots;
  int nr_links;

  nr_robots = kcd_get_nof_grps();
  nr_links = kcd_get_nof_mos();
  interm_res = MY_ALLOC(robot_vs_static_aabb_collision_p,nr_kcd_aabb_list);
  nof_interm_res = MY_ALLOC(int,nr_kcd_aabb_list);
  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      interm_res[i] = NULL;
      interm_res[i] = MY_ALLOC(robot_vs_static_aabb_collision,nr_aabb_hier_el[i]);
      nof_interm_res[i] = 0;
    }

  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      cand_max = 0;
      for(j=0;j<nr_aabb_hier_el[i];j++)
	cand_max += AABB_TREE[i][j].group_below_nr;

      if(max_nr < cand_max)
	max_nr = cand_max;
    }

  robots_vs_static_aabbs_in_col = NULL;
  robots_vs_static_aabbs_in_col = MY_ALLOC(list_of_robot_vs_static_aabb_collision,nr_links);
  nof_robots_vs_static_aabbs = MY_ALLOC(int,nr_links);

  for(place_link=0;place_link<nr_links;place_link++)
    {
      robots_vs_static_aabbs_in_col[place_link] = NULL;
      robots_vs_static_aabbs_in_col[place_link] =
	MY_ALLOC(robot_vs_static_aabb_collision,max_nr);
    }

  kcd_init_report();
  kcd_init_nearest_obstacle();

  for(i=0;i<nr_robots;i++)
    {
      kcd_init_movable_stuff_robot(i);
    }
}

void kcd_clean_up_mov_stuff()
{
  int i,j,cand_max,max_nr = 1;
  int place_link,nr_links;

  nr_links = kcd_get_nof_mos();
  for(i=0;i<nr_kcd_aabb_list;i++)
    {
      cand_max = 0;
      for(j=0;j<nr_aabb_hier_el[i];j++)
	cand_max += AABB_TREE[i][j].group_below_nr;

      if(max_nr < cand_max)
	max_nr = cand_max;
    }

  for(place_link=0;place_link<nr_links;place_link++)
    {
      MY_FREE(robots_vs_static_aabbs_in_col[place_link],robot_vs_static_aabb_collision,max_nr);
      robots_vs_static_aabbs_in_col[place_link] = NULL;
    }
  MY_FREE(robots_vs_static_aabbs_in_col,list_of_robot_vs_static_aabb_collision,nr_links);
  robots_vs_static_aabbs_in_col = NULL;
  MY_FREE(nof_robots_vs_static_aabbs,int,nr_links);
  nof_robots_vs_static_aabbs = NULL;
}


/* ************************************************************ *
 * ************************************************************ *
 * 
 * new function  allowing to add an obstacle dynamically to KCD
 * 
 * ************************************************************ *
 * ************************************************************ */

static void kcd_reinit_interm_res(int hh, int old_size, int new_size)
{
  interm_res[hh] = MY_REALLOC(interm_res[hh],robot_vs_static_aabb_collision,old_size,new_size);
  nof_interm_res[hh] = new_size;
}

/* only works for static objects added to scene after initialization */
void add_to_aabb_hierarchy()
{
  int nr_p3d_objs;
  int old_nof_objs;
  int pol_nr;
  int ith_so;
  int kcd_sml_prim_i;
  int kcd_big_prim_i;
  int i,j,hh;
  int bb;
  /* higher levels: */
  int nof_interm_levels;
  int nofNewBoxes;
  int nofbbs;
  int newb;
  int *kidList = NULL;
  int *AABBatPrevLevel = NULL;
  int *grp_nr = NULL;
//   kcd_bb_p aabbtreeroot;
//   kcd_bb_p *new_kid_array = NULL;
  int bb_id;
  int nofNewGroupAtThisLevel,nofNewGroupsAtLowerLevel,nofGroupsAtThisLevel,nofGroupsAtLowerLevel;
  /* root */
  int prev_aabb_nr,prevQuantity,q;
  int *prevArray = NULL;
  int *newArray = NULL;
//   int tel;

  /* start with first level */
  /* ************************************************************ */

  /* for each new static primitive */
  nr_p3d_objs =  kcd_nof_obst_to_add();
  // WAS:
  // old_nof_objs = kcd_get_nof_objects() - nr_p3d_objs;
  // NOW:
  old_nof_objs = kcd_get_nof_sos() - nr_p3d_objs;

  pol_nr = 0;
  for(i=0;i<nr_p3d_objs;i++)
    {
      // WAS:
      // ith_so = ith_static_obj[i] + old_nof_objs;
      // NOW:
      ith_so = ith_static_obj[i+old_nof_objs];

      kcd_sml_prim_i = kcd_o_prm_table[ith_so].sml_index;
      kcd_big_prim_i = kcd_o_prm_table[ith_so].big_index;

      for(j=kcd_sml_prim_i;j<=kcd_big_prim_i;j++)
	{
	  bb = kcd_obj_prim_table[j].kcd_prim_bb_id;
	  if(bb != -1)
	    {
	      /* insert AABB on lowest level */
	      insert_static_aabb(bb,0,nr_aabb_hier_el[0],NULL,0);
	      pol_nr++;
	    }
	}
    }

  kcd_reinit_interm_res(0,nr_aabb_hier_el[0]-pol_nr,nr_aabb_hier_el[0]);


  /* continue with higher levels */
  /* ************************************************************ */

  make_hash_table_on_bbs();

  AABBatPrevLevel = MY_ALLOC(int,pol_nr);
  kidList = MY_ALLOC(int,pol_nr);
  grp_nr = MY_ALLOC(int,pol_nr);

  nofNewBoxes = pol_nr;
  nofbbs = kcd_get_number_of_bbs();
  for(newb=0;newb<nofNewBoxes;newb++)
    {
      AABBatPrevLevel[newb] = nofbbs - nofNewBoxes + newb;
    }

  // for each of the levels 1..n of the hierarchy (n===nof_interm_levels), 
  //  where n is 1 less than the height of the AABB_TREE, do 

  nof_interm_levels = nr_kcd_aabb_list - 1 ;
  for(hh=1;hh<=nof_interm_levels;hh++)
  {
    // keep track of index of newly created AABB, for next level
    // (in particular, the indices of the new AABBs at level 0 are stored here)
    // for each of the AABBs added at the previous level
    for(newb=0;newb<nofNewBoxes;newb++)
    {
      // newb-th AABB added on previous level is put into kidList
      kidList[newb] = nr_aabb_hier_el[hh-1] - nofNewBoxes + newb;
      // create a new AABB and connect the new parent to its child
      grp_nr[newb] = kcd_determine_aabb_around_aabbs(&(kidList[newb]),1,hh-1);
    }
    
    // store new AABBs in the array of all_bbs
    make_hash_table_on_bbs();
    nofbbs = kcd_get_number_of_bbs();
    for(newb=0;newb<nofNewBoxes;newb++)
      {
	AABBatPrevLevel[newb] = nofbbs - nofNewBoxes + newb;
      }

    // for each of the new AABBs 
    nofGroupsAtThisLevel  = nr_aabb_hier_el[hh];
    nofGroupsAtLowerLevel = nr_aabb_hier_el[hh-1];

    if(hh != nof_interm_levels)
      {
	/* we add boxes to the current AABB level */
	for(newb=0;newb<nofNewBoxes;newb++)
	  {
	    // insert the parent into the AABB_TREE at its level
	    // insert_static_aabb(AABBatPrevLevel[newb],hh,grp_nr[newb],&(kidList[newb]),1);
	    nofNewGroupAtThisLevel = nofGroupsAtThisLevel + newb;
	    nofNewGroupsAtLowerLevel = nofGroupsAtLowerLevel - nofNewBoxes + newb;
	    insert_static_aabb(AABBatPrevLevel[newb],hh,nofNewGroupAtThisLevel,&nofNewGroupsAtLowerLevel,1);
	  }
	kcd_reinit_interm_res(hh,nr_aabb_hier_el[hh]-pol_nr,nr_aabb_hier_el[hh]);
      }
    else
      {
	/* we put new children in the root-box and resize the root-box */
	// collect current data in root
	prev_aabb_nr = AABB_TREE[hh][0].the_bb;
	prevQuantity = AABB_TREE[hh][0].group_below_nr;
	prevArray = AABB_TREE[hh][0].group_below_aabb_hier_el;

	// prepare new data for the root
	newArray = MY_ALLOC(int,prevQuantity+pol_nr);
	for(q=0;q<prevQuantity;q++)
	  {
	    newArray[q] = prevArray[q];
	  }
	for(q=0;q<pol_nr;q++)
	  {
	    newArray[prevQuantity + q]   = nofGroupsAtLowerLevel - nofNewBoxes + q;
	  }

	// reconstruct the root
	insert_static_aabb(prev_aabb_nr,hh,0,newArray,prevQuantity+pol_nr);

	// modify size of root AABB, in order to incapsulate the new AABBs
	for(q=0;q<pol_nr;q++)
	  {
	    bb_id = AABB_TREE[hh-1][AABB_TREE[hh][0].group_below_aabb_hier_el[prevQuantity+q]].the_bb;

	    if(all_bbs[prev_aabb_nr]->x1 > all_bbs[bb_id]->x1)
	      all_bbs[prev_aabb_nr]->x1 = all_bbs[bb_id]->x1;
	    if(all_bbs[prev_aabb_nr]->y1 > all_bbs[bb_id]->y1)
	      all_bbs[prev_aabb_nr]->y1 = all_bbs[bb_id]->y1;
	    if(all_bbs[prev_aabb_nr]->z1 > all_bbs[bb_id]->z1)
	      all_bbs[prev_aabb_nr]->z1 = all_bbs[bb_id]->z1;
	    
	    if(all_bbs[prev_aabb_nr]->x2 < all_bbs[bb_id]->x2)
	      all_bbs[prev_aabb_nr]->x2 = all_bbs[bb_id]->x2;
	    if(all_bbs[prev_aabb_nr]->y2 < all_bbs[bb_id]->y2)
	      all_bbs[prev_aabb_nr]->y2 = all_bbs[bb_id]->y2;
	    if(all_bbs[prev_aabb_nr]->z2 < all_bbs[bb_id]->z2)
	      all_bbs[prev_aabb_nr]->z2 = all_bbs[bb_id]->z2;

	  }

	// remember that the root AABB now contains boxes around several (>1) objects
	all_bbs[prev_aabb_nr]->ext_obj_id = -1;


      }

  }

  // clean up local allocations
  MY_FREE(AABBatPrevLevel,int,pol_nr);
  MY_FREE(kidList,int,pol_nr);
  MY_FREE(grp_nr,int,pol_nr);

}

