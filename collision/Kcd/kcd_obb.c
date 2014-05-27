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

#define  RELEVANT_SCALE_FACTOR  3.0     
#define  RELEVANT_DIV_FACTOR 0.3333     /*   1/RELEVANT_SCALE_FACTOR   */



static int stats_nof_solids = 0;
static int stats_nof_polyhs = 0;

int get_stats_nof_solids()
{
  return stats_nof_solids;
}

int get_stats_nof_polyhs()
{
  return stats_nof_polyhs;
}

int kcd_obb_is_leaf(kcd_bb *this_obb)
{
  return ((!this_obb->parent_bb)&&(!this_obb->children_bb));
}

/* **************************************************************************** *
 * Function:  kcd_put_obb_around_polyh()
 *    computes an OBB around the given primitive (polyhedron or solid)
 *    adds the OBB to the KCD data structure all_bbs
 * ARGS IN :  m3d_poly_it     external data structure containing the primitive
 *            ext_p_id        external identifier of the primitive
 *            is_mov_obj      TRUE if the primitive belongs to a movable
 *                            object, FALSE otherwise
 *            ext_obj_id      external identifier of the object to which the
 *                            primitive belongs
 * RETURNS :  the id of the obb, being the rank number of the pointer to the bb
 *            around the primitive in the array all_bbs
 * **************************************************************************** */
int kcd_put_obb_around_polyh(void *m3d_poly_it, int ext_p_id, int is_mov_obj, int ext_obj_id)
{
  int entity_type;
  double box_x = P3D_HUGE ,box_y = P3D_HUGE ,box_z =P3D_HUGE; //avoid warnings
  double box_a = P3D_HUGE ,box_b = P3D_HUGE,box_c= P3D_HUGE ;//avoid warnings
  double radius1, radius2;
  kcd_vector3 center;
  kcd_vector3 placed_center;
  kcd_vector3 evects[3];
  kcd_vector3 orient[3];
/*  kcd_matrix4 interm;*/
/*  int analyzer1,analyzer2;*/
  int poly_obb_nr = 0; // avoid warnings
  int small_volume_reached;   /* Modif. Carl on volume */
  double this_bb_volume;      /* Modif. Carl on volume */
  kcd_matrix4 *m3d_poly_it_pos0;

  entity_type = kcd_get_poly_entity_type(m3d_poly_it);
  if(entity_type==POLYHEDRON_ENTITY)
    {
      if(kcd_poly_is_convex(m3d_poly_it))
	{
	  entity_type = CONVEX_POLYHEDRON;
	}
      else
	{
	  entity_type = CONCAVE_POLYHEDRON;
	}
    }
  switch(entity_type)
    {
    case CONVEX_POLYHEDRON:
      stats_nof_polyhs++;
/* #ifdef CP_IS_SOLID */
      /* box size and placement */
/*       poly_obb_nr = enclose_convex_set_by_obb(m3d_poly_it,&box_x,&box_y,&box_z,center,evects); */
      /* independent of is_mov_obj: */
      /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
/*       p3d_xformVect(m3d_poly_it->pos0,evects[0],orient[0]);   */
/*       p3d_xformVect(m3d_poly_it->pos0,evects[1],orient[1]);   */
/*       p3d_xformVect(m3d_poly_it->pos0,evects[2],orient[2]);   */
/*       p3d_CxformPoint(m3d_poly_it->pos0,center,placed_center); */
      /* put obb away */
/*       poly_obb_nr = put_obb_away(box_x, box_y, box_z, placed_center, orient, m3d_poly_it,  */
/* 				 0,NULL,OBB_SPHERICAL,FALSE,is_mov_obj); */
      /* : independent of is_mov_obj */

/* #else */
      poly_obb_nr = enclose_convex_set_with_obb_tree(m3d_poly_it,ext_p_id,ext_obj_id,is_mov_obj);
/* #endif */
      break;
    case CONCAVE_POLYHEDRON:
      stats_nof_polyhs++;
      /* get box size, center, and orientation */
      poly_obb_nr = enclose_triangles_from_concave_with_obb_tree(m3d_poly_it,ext_p_id,ext_obj_id,is_mov_obj);
      break;
    case SPHERE_ENTITY:
      stats_nof_solids++;
      /* box size */
      kcd_get_solid_r1(m3d_poly_it,&box_x);
      /* begin modif. Carl on volume */
      this_bb_volume = 8.0 * box_x * box_x * box_x; 
      small_volume_reached = kcd_volume_smaller_than_user_defined_small_volume( this_bb_volume );  

      /* box center */
      center[0] = 0.0;
      center[1] = 0.0;
      center[2] = 0.0;
      /* box orientation */      /* who cares */
      evects[0][0]=1.0;
      evects[0][1]=0.0;
      evects[0][2]=0.0;
      evects[1][0]=0.0;
      evects[1][1]=1.0;
      evects[1][2]=0.0;
      evects[2][0]=0.0;
      evects[2][1]=0.0;
      evects[2][2]=1.0;
      
      /* independent of is_mov_obj: */
      /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
      if(is_mov_obj)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_mov_obj,&m3d_poly_it_pos0);

      kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
      kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
      kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
      kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);

      /* put obb away */  
      poly_obb_nr = put_obb_away(small_volume_reached, box_x, box_x, box_x, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_SPHERICAL,FALSE,is_mov_obj,TRUE);
      /* : independent of is_mov_obj */
	

      break;
    case CUBE_ENTITY:
      stats_nof_solids++;
      /* box size */
      kcd_get_solid_x(m3d_poly_it,&box_x);
      this_bb_volume = box_x * box_x * box_x; 
      box_x = box_x/2.0;
      /* begin modif. Carl on volume */
      small_volume_reached = kcd_volume_smaller_than_user_defined_small_volume( this_bb_volume );  

      /* box center */
      center[0] = 0.0;
      center[1] = 0.0;
      center[2] = 0.0;
      /* box orientation */      /* who cares */
      evects[0][0]=1.0;
      evects[0][1]=0.0;
      evects[0][2]=0.0;
      evects[1][0]=0.0;
      evects[1][1]=1.0;
      evects[1][2]=0.0;
      evects[2][0]=0.0;
      evects[2][1]=0.0;
      evects[2][2]=1.0;
      
      /* independent of is_mov_obj: */
      /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
      if(is_mov_obj)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_mov_obj,&m3d_poly_it_pos0);

      kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
      kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
      kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
      kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
      /* put obb away */
      poly_obb_nr = put_obb_away(small_volume_reached, box_x, box_x, box_x, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_SPHERICAL,FALSE,is_mov_obj,TRUE);
      /* :independent of is_mov_obj */
	
      break;
    case BOX_ENTITY:
      stats_nof_solids++;
      /* box size */
      kcd_get_solid_xyz(m3d_poly_it,&box_x,&box_y,&box_z);
      this_bb_volume = box_x * box_y * box_z;
      box_x = box_x / 2.0;
      box_y = box_y / 2.0;
      box_z = box_z / 2.0;
      /* begin modif. Carl on volume */
      small_volume_reached = kcd_volume_smaller_than_user_defined_small_volume( this_bb_volume );  

      /* box center */
      center[0] = 0.0;
      center[1] = 0.0;
      center[2] = 0.0;
      
      if(is_mov_obj)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_mov_obj,&m3d_poly_it_pos0);

      kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
	  
      if((box_x <= box_y) && (box_y <= box_z))
	{
	  box_a = box_x;
	  box_b = box_y;
	  box_c = box_z;	  
	  /* box orientation */
	  evects[2][0]=-1.0;
	  evects[2][1]=0.0;
	  evects[2][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[0][0]=0.0;
	  evects[0][1]=0.0;
	  evects[0][2]=1.0;
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
      else if((box_x <= box_z) && (box_z <= box_y))
	{
	  box_a = box_x;
	  box_b = box_z;
	  box_c = box_y;	  
	  /* box orientation */
	  evects[0][0]=0.0;
	  evects[0][1]=1.0;
	  evects[0][2]=0.0;
	  evects[2][0]=1.0;
	  evects[2][1]=0.0;
	  evects[2][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=0.0;
	  evects[1][2]=1.0;
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
      else if((box_y <= box_x) && (box_x <= box_z))
	{
	  box_a = box_y;
	  box_b = box_x;
	  box_c = box_z;	  
	  /* box orientation */
	  evects[1][0]=1.0;
	  evects[1][1]=0.0;
	  evects[1][2]=0.0;
	  evects[0][0]=0.0;
	  evects[0][1]=0.0;
	  evects[0][2]=1.0;
	  evects[2][0]=0.0;
	  evects[2][1]=1.0;
	  evects[2][2]=0.0;
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
      else if((box_y <= box_z) && (box_z <= box_x))
	{
	  box_a = box_y;
	  box_b = box_z;
	  box_c = box_x;	  
	  /* box orientation */
	  evects[2][0]=0.0;
	  evects[2][1]=-1.0;
	  evects[2][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=0.0;
	  evects[1][2]=1.0;
	  evects[0][0]=1.0;
	  evects[0][1]=0.0;
	  evects[0][2]=0.0;
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
      else if((box_z <= box_x) && (box_x <= box_y))
	{
	  box_a = box_z;
	  box_b = box_x;
	  box_c = box_y;	  
	  /* box orientation */
	  evects[2][0]=0.0;
	  evects[2][1]=0.0;
	  evects[2][2]=1.0;
	  evects[1][0]=-1.0;
	  evects[1][1]=0.0;
	  evects[1][2]=0.0;
	  evects[0][0]=0.0;
	  evects[0][1]=1.0;
	  evects[0][2]=0.0;	
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
      else if((box_z <= box_y) && (box_y <= box_x))
	{
	  box_a = box_z;
	  box_b = box_y;
	  box_c = box_x;	  
	  /* box orientation */
	  evects[2][0]=0.0;
	  evects[2][1]=0.0;
	  evects[2][2]=1.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[0][0]=1.0;
	  evects[0][1]=0.0;
	  evects[0][2]=0.0;
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);  
	}
	  
      if(box_b / box_a > RELEVANT_SCALE_FACTOR * box_c /  box_b)
	{
	  /* put obb away */
	  /* independent of is_mov_obj: */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_c, box_b, box_a, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_FLAT,FALSE,is_mov_obj,TRUE); 
	  /* :independent of is_mov_obj */
	}
      else if(box_b * RELEVANT_SCALE_FACTOR < box_c)
	{
	  /* independent of is_mov_obj: */
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_c, box_b, box_a, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_TALL,FALSE,is_mov_obj,TRUE); 
	  /* :independent of is_mov_obj */
	}
      else
	{
	  /* independent of is_mov_obj: */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_c, box_b, box_a, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_SPHERICAL,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	}
      break;
    case CYLINDER_ENTITY:
      stats_nof_solids++;
      /* box size */
      kcd_get_solid_r1(m3d_poly_it,&box_x);
      kcd_get_solid_h(m3d_poly_it,&box_z);
      box_z = box_z / 2.0;

      /* begin modif. Carl on volume */
      this_bb_volume = 8.0 * box_x * box_x * box_z; 
      small_volume_reached = kcd_volume_smaller_than_user_defined_small_volume( this_bb_volume );  

      /* box center */
      center[0] = 0.0;
      center[1] = 0.0;
      center[2] = 0.0;
      
      if(is_mov_obj)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_mov_obj,&m3d_poly_it_pos0);
      
      kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
	  
      if( (box_z / box_x) > RELEVANT_SCALE_FACTOR )
	{
	  /* box orientation */
	  evects[0][0]=0.0;
	  evects[0][1]=0.0;
	  evects[0][2]=1.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[2][0]=1.0;
	  evects[2][1]=0.0;
	  evects[2][2]=0.0;
	  /* independent of is_mov_obj: */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]); 
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);  
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]); 
	  /* watch out: vector 3 is X, vector 1 ix Z, vector 2 is Y */
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_z, box_x, box_x, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_TALL,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	  
	}
      else if( (box_z / box_x) < RELEVANT_SCALE_FACTOR )
	{
	  /* box orientation */
	  /* as expected */
	  evects[0][0]=1.0;
	  evects[0][1]=0.0;
	  evects[0][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[2][0]=0.0;
	  evects[2][1]=0.0;
	  evects[2][2]=1.0;
	  /* independent of is_mov_obj: */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_x, box_x, box_z, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_FLAT,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	}
      else  
	{
	  /* box orientation */
	  /* who cares */
	  evects[0][0]=1.0;
	  evects[0][1]=0.0;
	  evects[0][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[2][0]=0.0;
	  evects[2][1]=0.0;
	  evects[2][2]=1.0;
	  /* independent of is_mov_obj: */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached,box_x, box_x, box_z, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_SPHERICAL,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	}
	
      break;
    case CONE_ENTITY:
      stats_nof_solids++;
      /* box size */
      kcd_get_solid_r1(m3d_poly_it,&radius1);
      kcd_get_solid_r2(m3d_poly_it,&radius2);
      /* in fact, we assume radius2 > radius1 */
      if(radius2 > radius1)
	box_x = radius2;
      else
	box_x = radius1;

      kcd_get_solid_h(m3d_poly_it,&box_z);
      box_z = box_z / 2.0;
      /* begin modif. Carl on volume */
      this_bb_volume = 8.0 * box_x * box_x * box_z; 
      small_volume_reached = kcd_volume_smaller_than_user_defined_small_volume( this_bb_volume );  

      /* box center */
      center[0] = 0.0;
      center[1] = 0.0;
      center[2] = 0.0;

      if(is_mov_obj)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_mov_obj,&m3d_poly_it_pos0);
      
      kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
	  
      if( box_z / box_x > RELEVANT_SCALE_FACTOR )
	{
	  /* box orientation */
	  /* watch out: vector 3 is X, vector 1 ix Z, vector 2 is Y */
	  evects[0][0]=0.0;
	  evects[0][1]=0.0;
	  evects[0][2]=1.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[2][0]=1.0;
	  evects[2][1]=0.0;
	  evects[2][2]=0.0;
	  /* independent of is_mov_obj: */
	  /* we assume last line of m3d_poly_it->poly->pos == 0 0 0 1 */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_z, box_x, box_x, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_TALL,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	}
      else
	{
	  /* box orientation */
	  /* vector 1 is X, vector 2 is Y, vector 3 ix Z */
	  evects[0][0]=1.0;
	  evects[0][1]=0.0;
	  evects[0][2]=0.0;
	  evects[1][0]=0.0;
	  evects[1][1]=1.0;
	  evects[1][2]=0.0;
	  evects[2][0]=0.0;
	  evects[2][1]=0.0;
	  evects[2][2]=1.0;
	  /* independent of is_mov_obj: */
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[0],orient[0]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[1],orient[1]);
	  kcd_TransfoVect(*m3d_poly_it_pos0,evects[2],orient[2]);
	  /* put obb away */
	  poly_obb_nr = put_obb_away(small_volume_reached, box_x, box_x, box_z, placed_center, orient, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL,OBB_FLAT,FALSE,is_mov_obj,TRUE);
	  /* :independent of is_mov_obj */
	}
	
      break;
    case POLYHEDRON_ENTITY:
      PrintInfo(("error KCD (kcd_put_obb_around_polyh): polyhedron entity type should be CONVEX or CONCAVE \n"));
      break;
    default : PrintInfo(("error KCD: polyhedron entity type undefined \n"));
    }
  return poly_obb_nr;
}

static void generate_obb_tree_on_body_boxes(int all_bb_nr, int mo_id)
{
  int ext_o_id,sml_i,big_i;
/*  int poly_id;*/
  int i,j,noo = 0,nof_orig_obbs = 0;
  kcd_bb_p *the_orig_obbs = NULL;
  kcd_vector3 *the_points = NULL;
  int nb_points;
  int **the_facet_vertices = NULL;
  double bx,by,bz;
  kcd_vector3 eigenv[3];
  kcd_vector3 center;

  ext_o_id = kcd_mo_bb_table[mo_id].ext_o_id;
  nof_orig_obbs = kcd_get_nof_prm_of_ext_o(ext_o_id);

  if(nof_orig_obbs)
    {
  nb_points = 8*nof_orig_obbs;
  the_points = MY_ALLOC(kcd_vector3,nb_points);
  the_facet_vertices = MY_ALLOC(int *,nof_orig_obbs);

  the_orig_obbs = MY_ALLOC(kcd_bb_p,nof_orig_obbs);
  sml_i = kcd_o_prm_table[ext_o_id].sml_index;
  big_i = kcd_o_prm_table[ext_o_id].big_index;
  for(i=sml_i;i<=big_i;i++)
    {
      /* keep track of the original obbs */
      the_facet_vertices[noo] = MY_ALLOC(int,8);
      the_orig_obbs[noo] = all_bbs[kcd_poly_bb_table[i].bb_id];
      /* get size of OBB */
      bx = the_orig_obbs[noo]->d[0];
      by = the_orig_obbs[noo]->d[1];
      bz = the_orig_obbs[noo]->d[2];
      /* get orientation of OBB */
      kcd_vectCopy(the_orig_obbs[noo]->eigenv[0],eigenv[0]);
      kcd_vectCopy(the_orig_obbs[noo]->eigenv[1],eigenv[1]);
      kcd_vectCopy(the_orig_obbs[noo]->eigenv[2],eigenv[2]);
      /* get placement of OBB */
      center[0] = the_orig_obbs[noo]->center[0];
      center[1] = the_orig_obbs[noo]->center[1];
      center[2] = the_orig_obbs[noo]->center[2];
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
      the_points[0+noo*8][0] = center[0] - eigenv[0][0] - eigenv[1][0] - eigenv[2][0];
      the_points[0+noo*8][1] = center[1] - eigenv[0][1] - eigenv[1][1] - eigenv[2][1];
      the_points[0+noo*8][2] = center[2] - eigenv[0][2] - eigenv[1][2] - eigenv[2][2];
      /* point 1 */
      the_points[1+noo*8][0] = center[0] + eigenv[0][0] - eigenv[1][0] - eigenv[2][0];
      the_points[1+noo*8][1] = center[1] + eigenv[0][1] - eigenv[1][1] - eigenv[2][1];
      the_points[1+noo*8][2] = center[2] + eigenv[0][2] - eigenv[1][2] - eigenv[2][2];
      /* point 2 */
      the_points[2+noo*8][0] = center[0] + eigenv[0][0] - eigenv[1][0] + eigenv[2][0];
      the_points[2+noo*8][1] = center[1] + eigenv[0][1] - eigenv[1][1] + eigenv[2][1];
      the_points[2+noo*8][2] = center[2] + eigenv[0][2] - eigenv[1][2] + eigenv[2][2];
      /* point 3 */
      the_points[3+noo*8][0] = center[0] - eigenv[0][0] - eigenv[1][0] + eigenv[2][0];
      the_points[3+noo*8][1] = center[1] - eigenv[0][1] - eigenv[1][1] + eigenv[2][1];
      the_points[3+noo*8][2] = center[2] - eigenv[0][2] - eigenv[1][2] + eigenv[2][2];
      /* point 4 */
      the_points[4+noo*8][0] = center[0] - eigenv[0][0] + eigenv[1][0] - eigenv[2][0];
      the_points[4+noo*8][1] = center[1] - eigenv[0][1] + eigenv[1][1] - eigenv[2][1];
      the_points[4+noo*8][2] = center[2] - eigenv[0][2] + eigenv[1][2] - eigenv[2][2];
      /* point 5 */
      the_points[5+noo*8][0] = center[0] + eigenv[0][0] + eigenv[1][0] - eigenv[2][0];
      the_points[5+noo*8][1] = center[1] + eigenv[0][1] + eigenv[1][1] - eigenv[2][1];
      the_points[5+noo*8][2] = center[2] + eigenv[0][2] + eigenv[1][2] - eigenv[2][2];
      /* point 6 */
      the_points[6+noo*8][0] = center[0] + eigenv[0][0] + eigenv[1][0] + eigenv[2][0];
      the_points[6+noo*8][1] = center[1] + eigenv[0][1] + eigenv[1][1] + eigenv[2][1];
      the_points[6+noo*8][2] = center[2] + eigenv[0][2] + eigenv[1][2] + eigenv[2][2];
      /* point 7 */
      the_points[7+noo*8][0] = center[0] - eigenv[0][0] + eigenv[1][0] + eigenv[2][0];
      the_points[7+noo*8][1] = center[1] - eigenv[0][1] + eigenv[1][1] + eigenv[2][1];
      the_points[7+noo*8][2] = center[2] - eigenv[0][2] + eigenv[1][2] + eigenv[2][2];
	  
      for(j=0;j<8;j++)
	the_facet_vertices[noo][j] = noo*8+j+1;
	  
      noo++;
    }

  /* call OBB-tree construction on these points */
  enclose_body_obbs_with_obb_tree(all_bb_nr,nof_orig_obbs, the_orig_obbs, the_points, nb_points, the_facet_vertices);

  MY_FREE(the_points,kcd_vector3,nb_points);
  for(i=0;i<nof_orig_obbs;i++)
    MY_FREE(the_facet_vertices[i],int,8);
  MY_FREE(the_facet_vertices,int *,nof_orig_obbs);
  MY_FREE(the_orig_obbs,kcd_bb_p,nof_orig_obbs);
    }
}


/* KCD-API VERSION */
void kcd_obb_construct_on_links()
{
  int i,nof_mos;

  /* movable objects */
  nof_mos=kcd_get_nof_mos();
  for(i=0;i<nof_mos;i++)
    {
      generate_obb_tree_on_body_boxes(kcd_mo_bb_table[i].bb_id,i);
    }
}

