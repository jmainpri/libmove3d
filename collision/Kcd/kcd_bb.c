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



kcd_bb *kcd_get_the_kcd_bb(int rank_in_all_bbs)
{
  return all_bbs[rank_in_all_bbs];
}

void get_kcd_bb_aabb(int kcd_bb,double *x1,double *x2,double *y1,
		     double *y2,double *z1,double *z2)
{
      *x1 = all_bbs[kcd_bb]->x1;
      *x2 = all_bbs[kcd_bb]->x2;
      *y1 = all_bbs[kcd_bb]->y1;
      *y2 = all_bbs[kcd_bb]->y2;
      *z1 = all_bbs[kcd_bb]->z1;
      *z2 = all_bbs[kcd_bb]->z2;  
}

void get_kcd_bb_center(int all_bb_nr,
		       double *center_x,double *center_y,double *center_z)
{
    *center_x = all_bbs[all_bb_nr]->center[0];
    *center_y = all_bbs[all_bb_nr]->center[1];
    *center_z = all_bbs[all_bb_nr]->center[2];
}

void get_kcd_bb_sizes(int all_bb_nr,
		      double *d_x,double *d_y,double *d_z)
{
    *d_x = all_bbs[all_bb_nr]->d[0];
    *d_y = all_bbs[all_bb_nr]->d[1];
    *d_z = all_bbs[all_bb_nr]->d[2];
}

double get_kcd_bb_half_size(int all_bb_nr,int array_index)
{
  return all_bbs[all_bb_nr]->d[array_index];
}

int get_what_is_inside(int all_bb_nr)
{
  return all_bbs[all_bb_nr]->what_is_inside;
}

void set_what_is_inside(int all_bb_nr, int val)
{
  all_bbs[all_bb_nr]->what_is_inside = val;
}

int get_entity_type(int all_bb_nr)
{
  return all_bbs[all_bb_nr]->entity_type;
}

void set_entity_type(int all_bb_nr, int val)
{
  all_bbs[all_bb_nr]->entity_type = val;
}

int kcd_get_number_of_bbs()
{
  return nof_bbs;
}

int kcd_get_number_of_bbs_to_add()
{
  return nof_bbs_extra;
}


void init_all_bb()
{
  kcd_bb_p pile_it = first_bb;
  int i,nb_entries;

  if(all_bbs)
    {
      for(i=0;i<nof_bbs;i++)
	{
	  nb_entries = all_bbs[i]->nof_bbs;
	  if(all_bbs[i]->array_of_bbs)
	    MY_FREE(all_bbs[i]->array_of_bbs,kcd_bb *,nb_entries);
	  nb_entries = all_bbs[i]->nof_facet_vertices;
	  if(all_bbs[i]->facet_vertices)
	    MY_FREE(all_bbs[i]->facet_vertices,int,nb_entries);
	  nb_entries = all_bbs[i]->nof_children_bb;
	  if(all_bbs[i]->children_bb)
	    MY_FREE(all_bbs[i]->children_bb,kcd_bb *,nb_entries);
	}
      MY_FREE(all_bbs,kcd_bb,nof_bbs+nof_bbs_extra);
    }
  nof_bbs=0;
  nof_bbs_extra=0;
  all_bbs = NULL;

  while(pile_it != NULL)
    {
      first_bbs_pile = pile_it->next;
      MY_FREE(pile_it,kcd_bb,1);
      pile_it = first_bbs_pile;
    }
  last_bbs_pile = NULL;
  first_bbs_pile = NULL;
  first_bb = NULL;
  nr_added_on_pile = 0;
}

void kcd_clean_all_bb()
{
  kcd_bb_p pile_it = first_bb;
  int i,nb_entries;

  for(i=0;i<nof_bbs;i++)
    {
      nb_entries = all_bbs[i]->nof_bbs;
      if(all_bbs[i]->array_of_bbs)
	MY_FREE(all_bbs[i]->array_of_bbs,kcd_bb *,nb_entries);
      nb_entries = all_bbs[i]->nof_facet_vertices;
      if(all_bbs[i]->facet_vertices)
	MY_FREE(all_bbs[i]->facet_vertices,int,nb_entries);
      nb_entries = all_bbs[i]->nof_children_bb;
      if(all_bbs[i]->children_bb)
	MY_FREE(all_bbs[i]->children_bb,kcd_bb *,nb_entries);

      all_bbs[i]->nof_bbs = 0;
      all_bbs[i]->nof_facet_vertices = 0;
      all_bbs[i]->nof_children_bb = 0;
    }
  MY_FREE(all_bbs,kcd_bb_p,nof_bbs+nof_bbs_extra);
  all_bbs = NULL;
  nof_bbs_extra=0;
  nof_bbs=0;

  while(pile_it != NULL)
    {
      first_bbs_pile = pile_it->next;
      MY_FREE(pile_it,kcd_bb,1);
      pile_it = first_bbs_pile;
    }
  last_bbs_pile = NULL;
  first_bbs_pile = NULL;
  first_bb = NULL;
  nr_added_on_pile = 0;
}

/* hash table is: all_bbs
   just make more empty space */
void resize_hash_table_on_bbs(int extra_size)
{
  int i;
  kcd_bb_p pile_it = NULL; 
  
  if(extra_size > 0)
    {
      all_bbs = MY_REALLOC(all_bbs,kcd_bb_p,nof_bbs,nof_bbs+extra_size);

      pile_it = first_bb;
      for(i=0;i<nof_bbs;i++)
	{
	  all_bbs[i] = pile_it;
	  pile_it->bb_id_in_all_bbs = i;
	  pile_it = pile_it->next;
	}
      if(nof_bbs > 0)
	all_bbs[nof_bbs-1]->next = NULL;
      
      for(i=0;i<extra_size;i++)
	{
	  all_bbs[nof_bbs+i] = NULL;
	}
      nof_bbs_extra = extra_size;
    }
}

/* hash table is: all_bbs */
void make_hash_table_on_bbs()
{
  int i;
  kcd_bb_p pile_it = NULL; 

  if(nr_added_on_pile > nof_bbs_extra)
    {
      all_bbs = MY_REALLOC(all_bbs,kcd_bb_p,nof_bbs,nof_bbs+nr_added_on_pile);
      nof_bbs_extra = nr_added_on_pile - nof_bbs_extra;

      pile_it = first_bb;
      for(i=0;i<nof_bbs;i++)
	{
	  all_bbs[i] = pile_it;
	  pile_it->bb_id_in_all_bbs = i;
	  pile_it = pile_it->next;
	}
    }
  if(nof_bbs > 0)
    all_bbs[nof_bbs-1]->next = first_bbs_pile;

  pile_it = first_bbs_pile;
  for(i=0;i<nr_added_on_pile;i++)
    {
      all_bbs[nof_bbs+i] = pile_it;
      pile_it->bb_id_in_all_bbs = nof_bbs+i;
      pile_it = pile_it->next;
    }
  nof_bbs += nr_added_on_pile;
  nof_bbs_extra -= nr_added_on_pile;
  nr_added_on_pile = 0;
  first_bbs_pile = NULL;
}


int put_bb_in_all_bb(kcd_bb_p new_obb)
{
  if(first_bb == NULL)
    first_bb = new_obb;
  if(first_bbs_pile == NULL)
    first_bbs_pile = new_obb;

  new_obb->prev = last_bbs_pile;
  new_obb->next = NULL;

  if(last_bbs_pile != NULL)
    last_bbs_pile->next = new_obb;

  last_bbs_pile = new_obb;
  nr_added_on_pile++;
  return (nof_bbs+nr_added_on_pile);
}

void make_tree_connection(int child_rank_number,kcd_bb_p parent_p)
{
  last_bbs_pile->parent_bb = parent_p;
  parent_p->children_bb = MY_REALLOC(parent_p->children_bb,kcd_bb_p,
				     parent_p->nof_children_bb,parent_p->nof_children_bb+1);
  parent_p->nof_children_bb += 1;
  parent_p->children_bb[parent_p->nof_children_bb-1]=last_bbs_pile;
}


void make_obb_tree_connection(kcd_bb_p kid_p, kcd_bb_p parent_p)
{
  kid_p->parent_bb = parent_p;
  parent_p->children_bb = MY_REALLOC(parent_p->children_bb,kcd_bb_p,
				     parent_p->nof_children_bb,parent_p->nof_children_bb+1);
  parent_p->nof_children_bb += 1;
  parent_p->children_bb[parent_p->nof_children_bb-1]=kid_p;
}


int get_obbtype_of_j(int j)
{
  if(j < nof_bbs)
    {
      return all_bbs[j]->type_of_obb;      
    }
  else
    {
      PrintInfo(("Kcd (get_obbtype_of_j): overflow in list all_bb\n"));
      return -1;
    }
}




