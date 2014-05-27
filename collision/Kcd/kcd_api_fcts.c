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
#include "P3d-pkg.h"
#include "Collision-pkg.h"


/* ***************************************************** */
/* These are functions of KCD, interface to void *polyPt */
/* ***************************************************** */
// extern int get_p3d_id_from_input_index(int input_index);


/* ******************************************************** *
 * Function: cd_get_prim_abs_pos()
 *           returns the absolute placement *placement
 *            for the primitive primPt
 * ARGS IN : primPt     pointer to the user's primitive data
 *           can_move   TRUE for primitive belonging to a
 *                      movable object, FALSE otherwise
 * ARGS OUT: placement  absolute placement matrix
 * ******************************************************** */
void kcd_get_prim_abs_pos(void *primPt, int can_move, void *placement)
/*   -returns the absolute placement *placement for the primitive primPt */
{
  if(can_move)
    {
      /* polyh->poly->pos */
      p3d_get_poly_pos_of_polyh((p3d_poly*)primPt,(p3d_matrix4**)placement);
    }
  else
    {
      /* polyh->pos0 */
      p3d_get_poly_pos0((p3d_poly*)primPt,(p3d_matrix4**)placement);
    }
}

/* ******************************************************** *
 * Function: kcd_get_prim_rel_pos()
 *           returns the relative placement *placement
 *           for the primitive primPt w.r.t. the movable
 *           object it belongs to
 * ARGS IN : primPt     pointer to the user's primitive data
 * ARGS OUT: placement  relative placement matrix
 * PRECOND : primitive belongs to a movable object
 * ******************************************************** */
void kcd_get_prim_rel_pos(void *primPt, void *placement)
/*   - returns the relative placement *placement for the primitive primPt */
/*     w.r.t. the movable (!) object to which it belongs */
{
  /* polyh->pos0 */
  p3d_get_poly_pos_rel_jnt((p3d_poly*)primPt,(p3d_matrix4**)placement);
}


/* ******************************************************** *
 * Function: kcd_get_obj_abs_pos()
 *           returns the absolute placement *placement
 *           for the kcd_ext_o-th object defined by the 
 *           user through kcd_beg_obj()
 * ARGS IN : kcd_ext_o  external index of the object
 * ARGS OUT: placement  absolute placement matrix
 * PRECOND : primitive belongs to a movable object
 * ******************************************************** */
void kcd_get_obj_abs_pos(int kcd_ext_o, void *placement)
{
  int ext_o_id;

  /* this function gets jnt->pos */
  ext_o_id = get_p3d_id_from_input_index(kcd_ext_o);
  /* PrintInfo(("== mo_o_id %i\n",ext_o_id)); */
  p3d_get_mo_pos(ext_o_id,(p3d_matrix4**)placement);
}


/* ******************************************************** *
 * Function: kcd_get_pt()
 *           gets i-th point of polyhedron
 *           stores its X-, Y-, Z-coordinates in 3 double-s 
 * ARGS IN : polyPt  pointer to the external data structure
 *                   of a polyhedron
 *           i       rank number of the point
 * ARGS OUT: x,y,z   X-, Y-, Z-coordinates of i-th point of
 *                   polyhedron polyPt
 * ******************************************************** */
void kcd_get_pt(void *polyPt,int i,double *x,double *y,double *z)
{
  p3d_get_poly_pt((p3d_poly*)polyPt,i,x,y,z);
}

/* ******************************************************** *
 * Function: kcd_get_pt_arr()
 *           gets array of the points of a polyhedron
 *           stores its X-, Y-, Z-coordinates in 3 double-s 
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 * ARGS OUT: the_points  array of points, each point is 
 *                       given by an array of size 3 of 
 *                       doubles (the X-, Y-, Z-coordinates) 
 * ******************************************************** */
void kcd_get_pt_arr(void *polyPt,void *the_points)
{
  p3d_get_poly_point_array((p3d_poly*)polyPt,(p3d_vector3**)the_points);
}

/* ******************************************************** *
 * Function: kcd_get_nb_pts()
 *           returns the number of points of a polyhedron
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 * RETURNS : an unsigned integer, the number of points
 * ******************************************************** */
unsigned int kcd_get_nb_pts(void *polyPt)
{
  return p3d_get_nof_points_of_polyh((p3d_poly*)polyPt);
}

/* ******************************************************** *
 * Function: kcd_get_nb_fs()
 *           returns the number of facets of a polyhedron
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 * RETURNS : an unsigned integer, the number of facets
 * ******************************************************** */
unsigned int kcd_get_nb_fs(void *polyPt)
{
  return p3d_get_nof_faces_of_polyh((p3d_poly*)polyPt);
}

/* ******************************************************** *
 * Function: kcd_get_nb_pts_in_f()
 *           returns the number of points of a given 
 *           facet of a polyhedron
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 *           i           index of the facet (the i-th facet)
 * RETURNS : an unsigned integer, the number of vertices 
 *           of the i-th facet of the polyhedron
 * ******************************************************** */
unsigned int kcd_get_nb_pts_in_f(void *polyPt, int i)
{
  /* 0 < i <= total_number_of_faces(polyPt) */
  return p3d_get_nof_points_in_face_of_polyh((p3d_poly*)polyPt,i);
}

/* ******************************************************** *
 * Function: kcd_get_i_pt_in_f()
 *           returns the index j, so that the j-th point 
 *           in the array of points of the polyhedron is
 *           the pid-th point of the fid-th facet
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 *           fid         index of the facet (the i-th facet)
 *           pid         index of the point in the array of
 *                       points of the fid-th facet 
 * RETURNS : an integer, the index of the pid-th point of the 
 *           fid-th facet in the array of points of the 
 *           polyhedron
 * REMARKS : 0 < fid <= total_number_of_faces(polyPt)
 *           0 < pid <= total_number_of_points_in_face(fid)
 * ******************************************************** */
int kcd_get_i_pt_in_f(void *polyPt, int fid, int pid)
{
  return p3d_get_index_point_in_face_of_polyh((p3d_poly*)polyPt,fid,pid);
}

/* ******************************************************** *
 * Function: kcd_get_pt_in_f_arr()
 *           returns the array of indices, the i-th index 
 *           indicating the place of the i-th vertex of the
 *           given facet in the array of points of the 
 *           polyhedron
 * ARGS IN : polyPt      pointer to the external data 
 *                       structure of a polyhedron
 *           fid         index of the facet (the i-th facet)
 *           the_points  array of indices of the vertices of
 *                       the fid-th facet of the polyhedron,
 *                       each being an index in the array of
 *                       points of the polyhedron
 * REMARKS : 0 < fid <= total_number_of_faces(polyPt)
 *           0 < pid <= total_number_of_points_in_face(fid)
 * ******************************************************** */
void kcd_get_pt_in_f_arr(void *polyPt, int fid, void *the_points)
{
  p3d_get_points_in_face_of_polyh((p3d_poly*)polyPt,fid,(unsigned int**)the_points);
}

/* poly->entity_type */
/*! \fn  int kcd_get_poly_entity_type(void *polyPt)
\brief get the entity type of the given polyPt
\note The PolyPt will only be NULL if we calculate the distance between 
       two OBB's
\warning if polyPt == NULL KCD_OBB has to be returned otherwise KCD is not
 able to calculate the distances between the OBB's
*/ 

int kcd_get_poly_entity_type(void *polyPt)
{  
  if(polyPt == NULL)
    {
      // PrintInfo(("kcd_get_poly_entity_type: we return KCD_OBB \n"));
      return (KCD_OBB);
    }
  else
    {
      return (p3d_get_poly_entity_type((p3d_poly*)polyPt));
    }
}

/* void kcd_set_poly_entity_type(void *polyPt, int the_entity_type) */
/* { */
/*   p3d_poly_set_entity_type(polyPt,the_entity_type); */
/* } */

/* poly convexity test */
int kcd_poly_is_convex(void *polyPt)
{
/*   return p3d_get_poly_is_convex(polyPt); */
  int polyPt_is_convex;

  polyPt_is_convex = p3d_get_poly_is_convex((p3d_poly*)polyPt);
  if(polyPt_is_convex)
    p3d_poly_set_entity_type((p3d_poly*)polyPt,CONVEX_POLYHEDRON);
  else
    p3d_poly_set_entity_type((p3d_poly*)polyPt,CONCAVE_POLYHEDRON);
  return polyPt_is_convex;
}

/* poly-facet convexity test */
int kcd_facet_is_convex(void *polyPt, int fid)
{
  return p3d_get_facet_is_convex((p3d_poly*)polyPt,fid);
}

/* primitive data */
void kcd_get_solid_r1(void *polyPt,double *the_datum)
{
  p3d_get_prim_radius((p3d_poly*)polyPt,the_datum);
}
void kcd_get_solid_r2(void *polyPt,double *the_datum)
{
  p3d_get_prim_other_radius((p3d_poly*)polyPt,the_datum);
}
void kcd_get_solid_h(void *polyPt,double *the_datum)
{
  p3d_get_prim_height((p3d_poly*)polyPt,the_datum);
}
void kcd_get_solid_s(void *polyPt,double *the_datum)
{
  p3d_get_prim_sin_slope((p3d_poly*)polyPt,the_datum);
}
void kcd_get_solid_x(void *polyPt,double *x)
{
  p3d_get_prim_x_length((p3d_poly*)polyPt,x);
}
void kcd_get_solid_y(void *polyPt,double *y)
{
  p3d_get_prim_y_length((p3d_poly*)polyPt,y);
}
void kcd_get_solid_z(void *polyPt,double *z)
{
  p3d_get_prim_z_length((p3d_poly*)polyPt,z);
}
void kcd_get_solid_xyz(void *polyPt,double *x,double *y,double *z)
{
  p3d_get_prim_lengths((p3d_poly*)polyPt,x,y,z);
}

/* ******************************************************** *
 * Function:  kcd_get_aabb_on_mo()
 *            The user may specify a function computing the 
 *            AABB around a movable object.
 *            KCD will call the function of the user rather
 *            than its own function to recompute the AABB of
 *            a movable object after it has been moved.
 *            If the user does supply a function, it must
 *            be called here, the result must be stored in
 *            (x1,x2,...,z2) and kcd_get_aabb_on_mo() must 
 *            return TRUE.
 *            If the user does not supply a function, it
 *            suffices that kcd_get_aabb_on_mo() returns 
 *            FALSE.
 * ARGS IN :  kcd_ext_o    the movable object was the 
 *                         kcd_ext_o-th object given by 
 *                         the user with kcd_beg_obj()
 * ARGS OUT:  x1           smallest absolute value in 
 *                         X-direction of the vertices of
 *                         the AABB
 *            x2           largest absolute value in 
 *                         X-direction of the vertices of
 *                         the AABB 
 *            y1           smallest absolute value in 
 *                         Y-direction of the vertices of
 *                         the AABB
 *            y2           largest absolute value in 
 *                         Y-direction of the vertices of
 *                         the AABB 
 *            z1           smallest absolute value in 
 *                         Z-direction of the vertices of
 *                         the AABB
 *            z2           largest absolute value in 
 *                         Z-direction of the vertices of
 *                         the AABB 
 * RETURNS:   TRUE if x1,..,z2 is filled, FALSE otherwise
 * PRECOND:   the kcd_ext_o-th object is movable
 * ******************************************************** */
int kcd_get_aabb_on_mo(int kcd_ext_o,
			double *x1,double *x2,double *y1,
			double *y2,double *z1,double *z2)
{
  return p3d_get_aabb_on_mo(kcd_ext_o,x1,x2,y1,y2,z1,z2);
}

/* *********************************************************** *
 * Function: kcd_get_scene_size()
 *           gives an upper-bound of the size of the scene
 * ARGS OUT: x  size of the range of the scene in X-direction
 *           y  size of the range of the scene in Y-direction
 *           z  size of the range of the scene in Z-direction
 * *********************************************************** */
void kcd_get_scene_size(double *x,double *y,double *z)
{
  p3d_get_env_size(x,y,z);
}

