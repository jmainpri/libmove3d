#include "Util-pkg.h"
#include "Collision-pkg.h"


#define KCD_NPT 15
#define AABBvsOBBrate 0.9    

static double bb_volume_aabb(kcd_bb *abox)
{
  return (abox->x2-abox->x1)*(abox->y2-abox->y1)*(abox->z2-abox->z1);
}

static double bb_volume_obb(kcd_bb *abox)
{
  return abox->d[0]*abox->d[1]*abox->d[2]*8;
}

static void eval_best_bbox(kcd_bb *abox)
{
  double avolume = bb_volume_aabb(abox);
  double ovolume = bb_volume_obb(abox);
  if(avolume < ovolume)
    {
      abox->best_box_type = AABB_BOXTYPE;
    }
  else if(avolume*AABBvsOBBrate < ovolume)
    {
      abox->best_box_type = AABB_BOXTYPE;
    }
  else
    {
      abox->best_box_type = OBB_BOXTYPE;
    }

}

static void kcd_set_best_box_type(int bb_id)
{
  eval_best_bbox(all_bbs[bb_id]);
}




/* ********************************************************************************************** *
 * Function: kcd_put_aabb_around_polyh()
 *   creation of a AABB around a primitive (solid or polyhedron)
 * ARGS IN : m3d_poly_it       pointer to external data structure containing the primitive
 *           guess_of_poly_id  rank number of corresponding bb in all_bbs (OBB around
 *                             the primitive exists already)
 *           can_move          TRUE if primitive belongs to movable object, FALSE otherwise
 *           initialization    TRUE if AABB was never constructed before,
 *                             FALSE if AABB must be recomputed around (moved) movable object
 * PRECOND : kcd_obb_construct() was called before (hence, data structure already in place) 
 * ********************************************************************************************** */
void kcd_put_aabb_around_polyh(void *m3d_poly_it,int guess_of_poly_id, int can_move,int initialization)
{
  int entity_type,nb_points;
  double xmin,xmax,ymin,ymax,zmin,zmax;
  double center_x,center_y,center_z,d_x,d_y,d_z,a,a2,b,b2,c,c2;
/*  double dt=2*M_PI/KCD_NPT;*/
  double r1,r2,l,l2;
  kcd_vector3 *the_points;
  kcd_matrix4 *pos0;

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

  if(!initialization)
    kcd_get_prim_abs_pos(m3d_poly_it,can_move,&pos0);
  else if(can_move)
    kcd_get_prim_rel_pos(m3d_poly_it,&pos0);
  else
    kcd_get_prim_abs_pos(m3d_poly_it,can_move,&pos0);

  switch(entity_type)
    {
    case CONVEX_POLYHEDRON:
      kcd_get_pt_arr(m3d_poly_it,&the_points);
      nb_points = kcd_get_nb_pts(m3d_poly_it);
      enclose_set_by_aabb(the_points,nb_points,pos0, &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
      break;
    case CONCAVE_POLYHEDRON:
      kcd_get_pt_arr(m3d_poly_it,&the_points);
      nb_points = kcd_get_nb_pts(m3d_poly_it);
      enclose_set_by_aabb(the_points,nb_points,pos0, &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
      break;
    case SPHERE_ENTITY:
      get_kcd_bb_center(guess_of_poly_id,&center_x,&center_y,&center_z);
      get_kcd_bb_sizes(guess_of_poly_id,&d_x,&d_y,&d_z);
      xmin = center_x - d_x;
      xmax = center_x + d_x;
      ymin = center_y - d_y;
      ymax = center_y + d_y;
      zmin = center_z - d_z;
      zmax = center_z + d_z;
      break;
    case CUBE_ENTITY:
      the_points = MY_ALLOC(kcd_vector3,8);
      kcd_get_solid_x(m3d_poly_it,&a);
      a2 = a/2.0;
      /* the cube with size a2: */
      the_points[0][0] = -a2;
      the_points[0][1] = -a2;
      the_points[0][2] =  a2;

      the_points[1][0] = -a2;
      the_points[1][1] =  a2;
      the_points[1][2] =  a2;

      the_points[2][0] =  a2;
      the_points[2][1] =  a2;
      the_points[2][2] =  a2;

      the_points[3][0] =  a2;
      the_points[3][1] = -a2;
      the_points[3][2] =  a2;

      the_points[4][0] = -a2;
      the_points[4][1] = -a2;
      the_points[4][2] = -a2;

      the_points[5][0] = -a2;
      the_points[5][1] =  a2;
      the_points[5][2] = -a2;

      the_points[6][0] =  a2;
      the_points[6][1] =  a2;
      the_points[6][2] = -a2;
 
      the_points[7][0] =  a2;
      the_points[7][1] = -a2;
      the_points[7][2] = -a2;

      enclose_set_by_aabb(the_points,8,pos0,
			  &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
      MY_FREE(the_points,kcd_vector3,8);
      break;
    case BOX_ENTITY:
      the_points = MY_ALLOC(kcd_vector3,8);
      kcd_get_solid_x(m3d_poly_it,&a);
      a2 = a/2.0;
      kcd_get_solid_y(m3d_poly_it,&b);
      b2 = b/2.0;
      kcd_get_solid_z(m3d_poly_it,&c);
      c2 = c/2.0;
      /* box of size a2,b2,c2: */
      the_points[0][0] = -a2;
      the_points[0][1] = -b2;
      the_points[0][2] =  c2;

      the_points[1][0] = -a2;
      the_points[1][1] =  b2;
      the_points[1][2] =  c2;

      the_points[2][0] =  a2;
      the_points[2][1] =  b2;
      the_points[2][2] =  c2;

      the_points[3][0] =  a2;
      the_points[3][1] = -b2;
      the_points[3][2] =  c2;

      the_points[4][0] = -a2;
      the_points[4][1] = -b2;
      the_points[4][2] = -c2;

      the_points[5][0] = -a2;
      the_points[5][1] =  b2;
      the_points[5][2] = -c2;

      the_points[6][0] =  a2;
      the_points[6][1] =  b2;
      the_points[6][2] = -c2;

      the_points[7][0] =  a2;
      the_points[7][1] = -b2;
      the_points[7][2] = -c2;

      enclose_set_by_aabb(the_points,8,pos0,
			  &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
      MY_FREE(the_points,kcd_vector3,8);
      break;
    case CYLINDER_ENTITY:
      kcd_get_solid_h(m3d_poly_it,&l);
      kcd_get_solid_r1(m3d_poly_it,&r1);
      /* radius r1, length l   l2=l/2: */
      l2=l/2.0;
      enclose_cylinder_by_aabb(r1,l2,pos0,
			  &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
      break;
    case CONE_ENTITY:
      kcd_get_solid_h(m3d_poly_it,&l);
      l2=l/2.0;
      kcd_get_solid_r1(m3d_poly_it,&r1);
      kcd_get_solid_r2(m3d_poly_it,&r2);
      enclose_cone_by_aabb(r1,r2,l2,pos0,
			  &xmin,&xmax,&ymin,&ymax,&zmin,&zmax);
     break;
    case POLYHEDRON_ENTITY:
      PrintInfo(("error KCD (kcd_put_aabb_around_polyh): polyhedron entity type should be CONVEX or CONCAVE \n"));
      break;
    default : PrintInfo(("error KCD: polyhedron entity type undefined \n"));
    }
  put_aabb_away(guess_of_poly_id,xmin,xmax,ymin,ymax,zmin,zmax,TRUE);
}


/* **************************************************** *
 * Function:   kcd_aabb_construct()
 *    KCD API version of kcd_aabb_construct()
 *    computes AABB around each primitive
 *    computes the AABB hierarchy on static objects
 * **************************************************** */
void kcd_aabb_construct()
{
  int j,can_move,initialization=TRUE;
  void *m3d_poly_it = NULL;

  kcd_init_AABB_TREE();

  /* walk through all polyhedrons and solids */
  for(j=0;j<nof_bbs;j++)
    {
      m3d_poly_it = all_bbs[j]->pol;
      can_move = all_bbs[j]->is_robot_prim;
      /* call creation of AABB */
      if((m3d_poly_it)&&(all_bbs[j]->is_obb_root))
	{
	  kcd_put_aabb_around_polyh(m3d_poly_it,j,can_move,initialization);
	  kcd_set_best_box_type(j);
	}
    }

  create_aabb_hierarchy();
}


/* ************************************************************ *
 * ************************************************************ *
 * 
 * new function  allowing to add an obstacle dynamically to KCD
 * 
 * ************************************************************ *
 * ************************************************************ */

/* only works for static objects added to scene after initialization */
void kcd_aabb_add(int nof_bbs_to_add, int tot_nof_boxes)
{
  int first_new_box;
  int j;
  int can_move = FALSE;
  int initialization=TRUE;
  void *m3d_poly_it = NULL;

  // compute aabb around each new entry 
  /* walk through all newly added polyhedrons and solids */
  first_new_box = tot_nof_boxes - nof_bbs_to_add;
  for(j=first_new_box;j<tot_nof_boxes;j++)
    {
      /* prepare input data for AABB generation */
      m3d_poly_it = all_bbs[j]->pol;
      // obsolete:
      // can_move = all_bbs[j]->is_robot_prim;

      /* call creation of AABB */
      if((m3d_poly_it)&&(all_bbs[j]->is_obb_root))
	{
	  kcd_put_aabb_around_polyh(m3d_poly_it,j,can_move,initialization);
	  kcd_set_best_box_type(j);
	}
    }

  // modify the aabb_hierarchy (insert new AABBs at each of the levels)
  add_to_aabb_hierarchy();
}
