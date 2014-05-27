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


id_bb      *flat_obbs,    *spherical_obbs,    *tall_obbs;
int      nof_flat_obbs, nof_spherical_obbs, nof_tall_obbs;

kcd_bb_p   *all_bbs;
int      nof_bbs; /* nof_flat_obbs + nof_spherical_obbs + nof_tall_obbs */
int nof_bbs_extra; /* extra allocated memory in all_bbs list, just in case */

kcd_bb_p first_bb;
kcd_bb_p first_bbs_pile;
kcd_bb_p last_bbs_pile;
int nr_added_on_pile;

int get_nr_on_pile()
{
  return nr_added_on_pile;
}

void print_bb(kcd_bb_p abox)
{
  int i;

  PrintInfo((" ============== \n"));
  if((abox->current_status_obb != UNSET) || TRUE)
    {
      PrintInfo(("half edge lengths: %f, %f, %f\n",abox->d[0],abox->d[1],abox->d[2]));
      PrintInfo(("center: (%f,%f,%f)\n",abox->center[0],abox->center[1],abox->center[2]));
      PrintInfo(("first  eigenvector: (%f,%f,%f)\n",abox->eigenv[0][0],abox->eigenv[0][1],abox->eigenv[0][2]));
      PrintInfo(("second eigenvector: (%f,%f,%f)\n",abox->eigenv[1][0],abox->eigenv[1][1],abox->eigenv[1][2]));
      PrintInfo(("third  eigenvector: (%f,%f,%f)\n",abox->eigenv[2][0],abox->eigenv[2][1],abox->eigenv[2][2]));
      if(abox->reference_frame==WORK)
	PrintInfo(("reference_frame is WORK\n"));
      else if(abox->reference_frame==LOCAL)
	PrintInfo(("reference_frame is LOCAL\n"));
      else
	PrintInfo(("something WRONG with reference_frame\n"));
      if(abox->current_status_obb==UNCHANGED)
	PrintInfo(("current_status_obb is UNCHANGED\n"));
      else if(abox->current_status_obb==MOVED)
	PrintInfo(("current_status_obb is MOVED\n"));
      else
	PrintInfo(("something WRONG with current_status_obb\n"));
      PrintInfo(("sits around: %i (1=polyhedron, 2=solid)\n",abox->what_is_inside));
      if(abox->what_is_inside==WAS_POLYHEDRON)
	{
	  PrintInfo(("polyhedron\n"));
	}
      else if(abox->what_is_inside==WAS_SOLID)
	{
	  PrintInfo(("solid\n"));
	}
      PrintInfo(("nof_facet_vertices: %i\n",abox->nof_facet_vertices));
      for(i=0;i<abox->nof_facet_vertices;i++)
	{
	  PrintInfo(("facet_vertices[%i] = %i\n",i,abox->facet_vertices[i]));
	}
    }
  if((abox->current_status_aabb != UNSET) || TRUE)
    {
      PrintInfo(("AABB\n"));
      PrintInfo(("xmin=%f,xmax=%f,ymin=%f,ymax=%f,zmin=%f,zmax=%f\n",abox->x1,abox->x2,abox->y1,abox->y2,abox->z1,abox->z2));
      if(abox->current_status_aabb==UNCHANGED)
	PrintInfo(("current_status_aabb is UNCHANGED\n"));
      else if(abox->current_status_aabb==MOVED_BACKWARD)
	PrintInfo(("current_status_aabb is MOVED_BACKWARD\n"));
      else if(abox->current_status_aabb==MOVED_FORWARD)
	PrintInfo(("current_status_aabb is MOVED_FORWARD\n"));
      else if(abox->current_status_aabb==MOVED)
	PrintInfo(("current_status_aabb is MOVED\n"));
      else
	PrintInfo(("something WRONG with current_status_aabb\n"));    
    }
  PrintInfo((" ^^^^^^^^^^^^^^ \n"));
}

void print_bb_boxes()
{
  int i,nr;

  nr = nof_bbs;
  PrintInfo(("nof_bbs=%i\n",nof_bbs));
  for(i=0;i<nr;i++)
    print_bb(all_bbs[i]);
}


void init_kcd_obb()
{
  if(flat_obbs)
    MY_FREE(flat_obbs,id_bb,nof_flat_obbs);
  nof_flat_obbs = 0;
  flat_obbs=NULL;
  if(spherical_obbs)
    MY_FREE(spherical_obbs,id_bb,nof_spherical_obbs);
  nof_spherical_obbs=0;
  spherical_obbs=NULL;
  if(tall_obbs)
    MY_FREE(tall_obbs,id_bb,nof_tall_obbs);
  nof_tall_obbs=0;
  tall_obbs=NULL;

  init_all_bb();
}

/* cleans up data flat_obbs, spherical_obbs, and tall_obbs */
void kcd_clean_up_kcd_type_obbs()
{
  if(nof_flat_obbs > 0)
    {
      MY_FREE(flat_obbs,id_bb,nof_flat_obbs);
      nof_flat_obbs = 0;
    }
  if(nof_spherical_obbs > 0)
    {
      MY_FREE(spherical_obbs,id_bb,nof_spherical_obbs);
      nof_spherical_obbs=0;
    }
  if(nof_tall_obbs > 0)
    {
      MY_FREE(tall_obbs,id_bb,nof_tall_obbs);
      nof_tall_obbs=0;
    }
}

int add_flat_obb(int m3d_poly_it_id,int obb_type,int obb_ref_num)
{
  nof_flat_obbs++;
  flat_obbs=MY_REALLOC(flat_obbs,id_bb,nof_flat_obbs-1,nof_flat_obbs);
  flat_obbs[nof_flat_obbs-1].p3d_poly_id=m3d_poly_it_id;
  flat_obbs[nof_flat_obbs-1].obb_type=obb_type;
  flat_obbs[nof_flat_obbs-1].obb_ref_num_by_type=obb_ref_num;
  return nof_flat_obbs;
}

int add_spherical_obb(int m3d_poly_it_id,int obb_type,int obb_ref_num)
{
  nof_spherical_obbs++;
  spherical_obbs=MY_REALLOC(spherical_obbs,id_bb,nof_spherical_obbs-1,nof_spherical_obbs);
  spherical_obbs[nof_spherical_obbs-1].p3d_poly_id=m3d_poly_it_id;
  spherical_obbs[nof_spherical_obbs-1].obb_type=obb_type;
  spherical_obbs[nof_spherical_obbs-1].obb_ref_num_by_type=obb_ref_num;
  return nof_spherical_obbs;
}

int add_tall_obb(int m3d_poly_it_id,int obb_type,int obb_ref_num)
{
  nof_tall_obbs++;
  tall_obbs=MY_REALLOC(tall_obbs,id_bb,nof_tall_obbs-1,nof_tall_obbs);
  tall_obbs[nof_tall_obbs-1].p3d_poly_id=m3d_poly_it_id;
  tall_obbs[nof_tall_obbs-1].obb_type=obb_type;
  tall_obbs[nof_tall_obbs-1].obb_ref_num_by_type=obb_ref_num;
  return nof_tall_obbs;
}

void clean_up_kcd_obb()
{
  init_kcd_obb();
}

int put_obb_away(int volume_is_too_small, double box_1, double box_2, double box_3, kcd_vector3 center, kcd_vector3 directions[3], void *m3d_poly_it, int ext_p_id, int ext_obj_id, int nof_the_vertices, int *the_vertices, int OBB_SHAPE, int leaf_value,int is_robot, int IS_OBB_ROOT)
{
  kcd_bb_p new_obb = NULL;
  int obb_ref_num=1 ,i,poly_entity_type=1; //set to 1 to avoid warning
/*  double new_volume;*/
  double surf;

  new_obb = MY_ALLOC(kcd_bb,1);

  new_obb->is_aabb_tree_leaf = leaf_value;
  new_obb->best_box_type = OBB_BOXTYPE;
  new_obb->is_obb_root = IS_OBB_ROOT;
  new_obb->type_of_obb = OBB_SHAPE;
  new_obb->d[0] = box_1;
  new_obb->d[1] = box_2;
  new_obb->d[2] = box_3;
  if(EQ(box_1,0.0))
    {
      surf = 4.0 * box_2 * box_3;
      new_obb->volume = surf * sqrt(surf);
    }
  else if(EQ(box_2,0.0))  
    { 
      surf = 4.0 * box_1 * box_3;
      new_obb->volume = surf * sqrt(surf);
    }
  else if(EQ(box_3,0.0)) 
    {
      surf = 4.0 * box_1 * box_2;
      new_obb->volume = surf * sqrt(surf);
    }
  else    
    {
      new_obb->volume = 8.0 * box_1 * box_2 * box_3;
    }
  new_obb->center[0] = center[0];
  new_obb->center[1] = center[1];
  new_obb->center[2] = center[2];
  new_obb->eigenv[0][0] = directions[0][0];
  new_obb->eigenv[0][1] = directions[0][1];
  new_obb->eigenv[0][2] = directions[0][2];
  new_obb->eigenv[1][0] = directions[1][0];
  new_obb->eigenv[1][1] = directions[1][1];
  new_obb->eigenv[1][2] = directions[1][2];
  new_obb->eigenv[2][0] = directions[2][0];
  new_obb->eigenv[2][1] = directions[2][1];
  new_obb->eigenv[2][2] = directions[2][2]; 
  new_obb->reference_frame = WORK;
  new_obb->current_status_obb = UNCHANGED;
  new_obb->parent_bb = NULL;
  new_obb->children_bb = NULL;
  new_obb->nof_children_bb = 0;
  /* B Kineo Carl 25.02.2002 */
  new_obb->ext_pol_id = ext_p_id;
  /* E Kineo Carl 25.02.2002 */
  /* B Kineo Carl 27.02.2002 */
  new_obb->ext_obj_id = ext_obj_id;
  /* E Kineo Carl 27.02.2002 */
  if(m3d_poly_it)
    {
      poly_entity_type = kcd_get_poly_entity_type(m3d_poly_it);
#ifdef CP_IS_SOLID
      if((poly_entity_type==POLYHEDRON_ENTITY) ||
         (poly_entity_type==CONCAVE_POLYHEDRON) )
#else
      if((poly_entity_type==POLYHEDRON_ENTITY) ||
         (poly_entity_type==CONVEX_POLYHEDRON) || 
         (poly_entity_type==CONCAVE_POLYHEDRON) )
#endif
      new_obb->what_is_inside = WAS_POLYHEDRON;
    else
      new_obb->what_is_inside = WAS_SOLID;
    }
  else
    {
      new_obb->what_is_inside = WAS_BBS;
    }

  if(volume_is_too_small)
    {
    new_obb->entity_type = KCD_SMALL_VOLUME_BOX;
    }
  else if(m3d_poly_it)
    new_obb->entity_type = poly_entity_type;
  else
    new_obb->entity_type = WAS_BBS;
  new_obb->pol = m3d_poly_it;
  new_obb->nof_facet_vertices = nof_the_vertices;
  if(nof_the_vertices == 0)
    {
      new_obb->facet_vertices = NULL;
    }
  else
    {
      new_obb->facet_vertices = MY_ALLOC(int,nof_the_vertices);
      for(i=0;i<nof_the_vertices;i++)
	{
	  new_obb->facet_vertices[i] = the_vertices[i];
	}
    }
  new_obb->interesting_seed = 0;
  new_obb->is_robot_prim = is_robot;
  new_obb->current_status_aabb = UNSET;

  new_obb->nof_bbs = 0;
  new_obb->array_of_bbs = NULL;

  switch(OBB_SHAPE){
  case OBB_TALL:
    obb_ref_num = put_bb_in_all_bb(new_obb);
    break;
  case OBB_SPHERICAL:
    obb_ref_num = put_bb_in_all_bb(new_obb);
    break;
  case OBB_FLAT:
    obb_ref_num = put_bb_in_all_bb(new_obb);
    break;
  default:
    PrintInfo(("ERROR (put_obb_away): OBB has no type\n"));
    break;
  }

  return obb_ref_num-1; /* all_bb[obb_ref_num-1] is the new (o)bb */

}

int put_new_aabb_away(int ext_obj_id, int is_aabb_tree_leaf,double x1, double x2, 
		       double y1, double y2, double z1, double z2,
		      int *the_aabb_group, int the_aabb_group_nofels, int hierheight)
{
  kcd_bb_p group_bb = MY_ALLOC(kcd_bb,1);
  int kid,aabb_ref_num = 0, bb_id;

      group_bb->best_box_type = AABB_BOXTYPE;

      /* put AABB data in */
      /* B Kineo Carl 27.02.2002 */
      group_bb->ext_obj_id = ext_obj_id;
      /* E Kineo Carl 27.02.2002 */

      group_bb->is_aabb_tree_leaf = is_aabb_tree_leaf;
      group_bb->x1 = x1;
      group_bb->x2 = x2;
      group_bb->y1 = y1;
      group_bb->y2 = y2;
      group_bb->z1 = z1;
      group_bb->z2 = z2;

      /* put children's data in */
      group_bb->what_is_inside = WAS_BBS ;    
      group_bb->entity_type = KCD_AABB ;
      group_bb->nof_facet_vertices = 0;
      group_bb->facet_vertices = NULL;
      group_bb->pol = NULL;
      group_bb->nof_bbs = the_aabb_group_nofels;
      group_bb->array_of_bbs = MY_ALLOC(kcd_bb_p,the_aabb_group_nofels);
      for(kid=0;kid<the_aabb_group_nofels;kid++)
	{
	  bb_id = kcd_get_the_bb(hierheight, the_aabb_group[kid]);  
	  group_bb->array_of_bbs[kid] = kcd_get_the_kcd_bb(bb_id);
	}
      /* fill up obb fields with uninteresting info */
      group_bb->best_box_type = AABB_BOXTYPE;
      group_bb->parent_bb = NULL;
      group_bb->nof_children_bb = 0;
      group_bb->children_bb = NULL;
      group_bb->d[0] = 0.0;
      group_bb->d[1] = 0.0;
      group_bb->d[2] = 0.0;
      group_bb->volume = 0.0;
      group_bb->center[0] = 0.0;
      group_bb->center[1] = 0.0;
      group_bb->center[2] = 0.0;
      group_bb->eigenv[0][0] = 0.0;
      group_bb->eigenv[0][1] = 0.0;
      group_bb->eigenv[0][2] = 0.0;
      group_bb->eigenv[1][0] = 0.0;
      group_bb->eigenv[1][1] = 0.0;
      group_bb->eigenv[1][2] = 0.0;
      group_bb->eigenv[2][0] = 0.0;
      group_bb->eigenv[2][1] = 0.0;
      group_bb->eigenv[2][2] = 0.0;


  aabb_ref_num = put_bb_in_all_bb(group_bb);
  return aabb_ref_num;
}

/* ****************************************************************** * 
 * Function :   put_aabb_away()
 *              adds AABB information to an already existing entry
 *              in  all_bbs  (for an OBB, constructed earlier)
 * ARGS IN :    obb_nr              index of BB in all_bbs
 *              x1,x2,y1,y2,z1,z2   representation of size of AABB
 *              is_leaf             TRUE if AABB is leaf in AABB_TREE
 *                                  (i.e. AABB sits around primitive)
 * PRECOND :    entry with index  obb_nr  exists in  all_bbs 
 *              is_leaf  should be TRUE
 * ****************************************************************** */
void put_aabb_away(int obb_nr,
		   double x1, double x2, double y1, double y2, 
		   double z1,double z2, int is_leaf)
{
  if(is_leaf)
    {
      /* add AABB data */
      all_bbs[obb_nr]->x1 = x1;
      all_bbs[obb_nr]->x2 = x2;
      all_bbs[obb_nr]->y1 = y1;
      all_bbs[obb_nr]->y2 = y2;
      all_bbs[obb_nr]->z1 = z1;
      all_bbs[obb_nr]->z2 = z2;
      all_bbs[obb_nr]->current_status_aabb = UNCHANGED;
      all_bbs[obb_nr]->is_aabb_tree_leaf = is_leaf;
      /* rest was filled by put_obb_away() */
    }
  else
    {
      PrintInfo(("Error: put_aabb_away tries to put non-leaf-aabb in database \n"));
    }
}
