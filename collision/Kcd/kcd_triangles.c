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
//#include "P3d-pkg.h"
#include "Collision-pkg.h"


/* data structures for polygon triangulation */
typedef enum
{
	OXY,
	OYZ,
	OXZ
} kcd_projection_type;

typedef struct kcd_tri_vertex
{
  double x;
  double y;  /* projected coordinates of point */
  int ref;     /* index of this vertex in facet of original polyhedron */
  int p3d_ref; /* index of this vertex in description of p3d-polyhedron */
} kcd_tri_vertex;

/* global variables for polygon triangulation */
static kcd_projection_type kcd_projection = OXY;
static kcd_tri_vertex *kcd_tri_vertices = NULL;
static int kcd_size_tri_vertices = 0;
static int nr_vertices = 0;
static output_triangle *list_of_triangles = NULL;

/* ****************************************************************************************** *
   Function: kcd_tri_get_normal()
             computes the normal of the given polygonal facet
   ARGS IN : the_points   all points of a polyhedron
             facet_i_pts  indices of entries of facet-points in array the_points
             nb_points    number of points in the facet
   ARGS OUT: A,B,C        the normal of the facet
   RETURNS : TRUE in case of success, FALSE in case of facet with normal equal to zero vector
 * ****************************************************************************************** */
int kcd_tri_get_normal(kcd_vector3 *the_points, int *facet_i_pts, int nb_points, 
		       double *A, double *B, double *C)
{
  int i,k,l,m,t = 0;
  double norm = 0.0;
  kcd_vector3 u1_point, u2_point;

  while( ( norm == 0.0 ) && ( t < nb_points-2 ) )
    {
      k = facet_i_pts[t]   - 1 ;
      l = facet_i_pts[t+1] - 1 ;
      m = facet_i_pts[t+2] - 1 ;
      /* on determine les deux premieres arretes de la face */
      for(i=0;i<3;i++)
	{
	  u1_point[i] = the_points[l][i] - the_points[k][i];
	  u2_point[i] = the_points[m][i] - the_points[l][i];
	}
      
      
      /* on calcul la normale a partir du produit vectoriel des vecteurs unitaires des arretes */
      *A = u1_point[1]*u2_point[2]-u1_point[2]*u2_point[1];
      *B = u1_point[2]*u2_point[0]-u1_point[0]*u2_point[2];
      *C = u1_point[0]*u2_point[1]-u1_point[1]*u2_point[0];
      
      norm = (*A) * (*A) + (*B) * (*B) + (*C) * (*C);
      t++; /* on essaie tous les triples de sommets avant de conclure que la face est anormale */
    }
  if (norm == 0.0)
    { 
      PrintInfo(("\nKCD Error: facet has normal zero vector\n"));
      return FALSE;
    }
  else
    {
      norm = sqrt(norm);
      (*A) = (*A) / norm;
      (*B) = (*B) / norm;
      (*C) = (*C) / norm;
      return TRUE;
    }
}

int kcd_tri_get_a_triangle_vertex(int triangle_nr,int vertex_nr)
{
  return list_of_triangles[triangle_nr][vertex_nr];
}

static int kcd_left(int poly_is_ccw,double A,double B,double C,double x,double y)
{
  if((double)poly_is_ccw*(A*x+B*y+C) > -FUZZ)
    return TRUE;
  else
    return FALSE;
}

static void clean_up_list_of_triangles(int triangle_list_size)
{
  MY_FREE(list_of_triangles,output_triangle,triangle_list_size);
  list_of_triangles = NULL;
}

static void kcd_tri_clean()
{
  if(kcd_tri_vertices)
    MY_FREE(kcd_tri_vertices,kcd_tri_vertex,kcd_size_tri_vertices);
  kcd_tri_vertices = NULL;
  nr_vertices = 0;
  kcd_size_tri_vertices = 0;
  kcd_projection = OXY;
}

static int kcd_tri_project_facet(kcd_vector3 *the_points, int *facet_i_pts, int nr_points)
{
  int poly_is_ccw = 1; /* == 1 if ccw, == -1 if cw */
  double A=0.0,B=0.0,C=0.0;
  int i;
//  double orig_x, orig_y, orig_z;
  double area = 0.0, x,y;

  kcd_tri_vertices = NULL;
  /* set global variable */
  nr_vertices = nr_points;

  /* make space for projected vertices */
  kcd_tri_vertices = MY_ALLOC(kcd_tri_vertex,nr_points);
  kcd_size_tri_vertices = nr_points;

  /* plane through facet: A.X + B.Y + C.Z + D = 0, D not needed below */
  if(! kcd_tri_get_normal(the_points,facet_i_pts,nr_points,&A,&B,&C) )
    { 
      A = B = C = 0.0;  /*error*/
      PrintInfo(("ERROR in kcd_tri_project_facet\n"));
    }
  /* determine the best projection to use */
  if(fabs(A) > fabs(B))
    if(fabs(A) > fabs(C))
      kcd_projection = OYZ;
    else
      kcd_projection = OXY;
  else
    if(fabs(B) > fabs(C))
      kcd_projection = OXZ;
    else
      kcd_projection = OXY;

  /* project all vertices of current facet according to best projection */
  switch(kcd_projection){
  case OXY: for(i=0;i<nr_points;i++)
    {
      kcd_tri_vertices[i].ref = i+1;
      kcd_tri_vertices[i].x = the_points[facet_i_pts[i]-1][0];
      kcd_tri_vertices[i].y = the_points[facet_i_pts[i]-1][1];
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      kcd_tri_vertices[i].p3d_ref = facet_i_pts[i];
    }
    break;
  case OXZ: for(i=0;i<nr_points;i++)
    {
      kcd_tri_vertices[i].ref = i+1;
      kcd_tri_vertices[i].x = the_points[facet_i_pts[i]-1][0];
      kcd_tri_vertices[i].y = the_points[facet_i_pts[i]-1][2];
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      kcd_tri_vertices[i].p3d_ref = facet_i_pts[i];
    }
    break;
  case OYZ: for(i=0;i<nr_points;i++)
    {
      kcd_tri_vertices[i].ref = i+1;
      kcd_tri_vertices[i].x = the_points[facet_i_pts[i]-1][1];
      kcd_tri_vertices[i].y = the_points[facet_i_pts[i]-1][2];
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      kcd_tri_vertices[i].p3d_ref = facet_i_pts[i];
    }
    break;
  }
  /* compute twice the area of the projected polygon */
  area = 0.0;
  x = kcd_tri_vertices[0].x;
  y = kcd_tri_vertices[0].y;
  for(i=2;i<nr_points;i++)
    area +=   (kcd_tri_vertices[i-1].x - x) 
            * (kcd_tri_vertices[i].y   - y)
            - (kcd_tri_vertices[i-1].y - y) 
            * (kcd_tri_vertices[i].x   - x);
  if(area < 0.0)
    poly_is_ccw = -1; /* else: poly_is_ccw = 1 */
  return poly_is_ccw;
}

static double kcd_tri_twice_the_triangle_area(int vnr_1,int vnr_2,int vnr_3)
{
  return (kcd_tri_vertices[vnr_2].x - kcd_tri_vertices[vnr_1].x)
        *(kcd_tri_vertices[vnr_3].y - kcd_tri_vertices[vnr_1].y) - 
         (kcd_tri_vertices[vnr_2].y - kcd_tri_vertices[vnr_1].y)
        *(kcd_tri_vertices[vnr_3].x - kcd_tri_vertices[vnr_1].x);
}

static int convex(int vnr_1, int vnr_2, int vnr_3, int poly_is_ccw)
{
  double t;
  int r;

  t = poly_is_ccw * kcd_tri_twice_the_triangle_area(vnr_1,vnr_2,vnr_3);
  if (t > FUZZ)
    r=1;
  else if (t < -FUZZ)
    r=0;
  else
    r=-1;

  return r;
}

static int diagonal(int vnr_1, int vnr_3, int poly_is_ccw)
{
  int res = TRUE;
  int vnr_2 = MOD(vnr_1 + 1,nr_vertices);
  struct { double A,B,C;} ac,cb,ba;
  double x,y;
  int i,im;

  res = convex(vnr_1,vnr_2,vnr_3,poly_is_ccw);
  if(res == -1)
    res = TRUE;
  else if(res == 0)
    res = FALSE;
  else
    {
      res = TRUE;
      ba.A=kcd_tri_vertices[vnr_3].y - kcd_tri_vertices[vnr_1].y;
      ba.B=kcd_tri_vertices[vnr_1].x - kcd_tri_vertices[vnr_3].x;
      ba.C= -ba.A*kcd_tri_vertices[vnr_1].x - ba.B*kcd_tri_vertices[vnr_1].y;
      ac.A=kcd_tri_vertices[vnr_1].y - kcd_tri_vertices[vnr_2].y;
      ac.B=kcd_tri_vertices[vnr_2].x - kcd_tri_vertices[vnr_1].x;
      ac.C= -ac.A*kcd_tri_vertices[vnr_2].x - ac.B*kcd_tri_vertices[vnr_2].y;
      cb.A=kcd_tri_vertices[vnr_2].y - kcd_tri_vertices[vnr_3].y;
      cb.B=kcd_tri_vertices[vnr_3].x - kcd_tri_vertices[vnr_2].x;
      cb.C= -cb.A*kcd_tri_vertices[vnr_3].x - cb.B*kcd_tri_vertices[vnr_3].y;      
      for(i=MOD(vnr_3+1,nr_vertices);(res)&&(MOD(i,nr_vertices)!=vnr_1);i++)
	{ 
	  im = MOD(i,nr_vertices);
	  if(!((kcd_tri_vertices[vnr_1].p3d_ref == kcd_tri_vertices[im].p3d_ref) ||
	     (kcd_tri_vertices[vnr_2].p3d_ref == kcd_tri_vertices[im].p3d_ref) ||
	     (kcd_tri_vertices[vnr_3].p3d_ref == kcd_tri_vertices[im].p3d_ref) ) )
	    {
	      x = kcd_tri_vertices[im].x;
	      y = kcd_tri_vertices[im].y;
	      if(kcd_left(poly_is_ccw,ba.A,ba.B,ba.C,x,y) &&
		 kcd_left(poly_is_ccw,ac.A,ac.B,ac.C,x,y) &&
		 kcd_left(poly_is_ccw,cb.A,cb.B,cb.C,x,y))
		res = FALSE;
	    }
	}
    }

  return res;
}

int kcd_tri_angulate_facet_p(kcd_vector3 *the_points,int *facet_i_pts,int nb_points,int nr_triangles,
			tricd_triangulation *tricd_decomp_p)
{
  int ok = FALSE;
  output_triangle_p *triangle_id_list_p = NULL;

  ok = kcd_tri_angulate_facet2(the_points,facet_i_pts,nb_points,nr_triangles,
		      tricd_decomp_p, triangle_id_list_p, FALSE);
  return ok;
}

int kcd_tri_angulate_facet_p_i(kcd_vector3 *the_points,int *facet_i_pts,int nb_points,int nr_triangles,
			tricd_triangulation *tricd_decomp_p,output_triangle_p *triangle_id_list_p)
{
//  int i,j; /* test */
  int ok = FALSE;

  ok = kcd_tri_angulate_facet2(the_points,facet_i_pts,nb_points,nr_triangles,
		      tricd_decomp_p, triangle_id_list_p, TRUE);

  return ok;
}

/* input:  polyhedron facet (a polygon) as described in Move3D */
/* output: list of triangles defined by the coordinates of their vertices */
int kcd_tri_angulate_facet2(kcd_vector3 *the_points,int *facet_i_pts,int nb_points,int nr_triangles,
			tricd_triangulation *tricd_decomp_p, 
			output_triangle_p *triangle_id_list_p,int fill_triangle_id_list)
{
  int nof_triangles,i,j,triangle_id = 0,ok;
//  double x,y,z;
  
  ok = kcd_tri_angulate_simple_polygon(the_points,facet_i_pts,nb_points,&nof_triangles);
  /* ok = kcd_tri_angulate_simple_polygon(m3d_polyh,fnum,&nof_triangles); */

/*   if(nof_triangles != nr_triangles) */
/*     PrintInfo(("WAGRHEGYU\n")); */
  if(ok)
    {
      if(fill_triangle_id_list)
	{
	  /* transform to list of triangle vertices */
	  for(i=0;i<nr_triangles;i++)
	    {
	      for(j=0;j<3;j++)
		{
		  triangle_id = kcd_tri_get_a_triangle_vertex(i,j);
		  (*triangle_id_list_p)[i][j] = triangle_id;
		  (*tricd_decomp_p)[i][j][0]=the_points[triangle_id-1][0];
		  (*tricd_decomp_p)[i][j][1]=the_points[triangle_id-1][1];
		  (*tricd_decomp_p)[i][j][2]=the_points[triangle_id-1][2];
		}
	    }
	}
      else
	{
	  for(i=0;i<nr_triangles;i++)
	    {
	      for(j=0;j<3;j++)
		{
		  triangle_id = kcd_tri_get_a_triangle_vertex(i,j);
		  (*tricd_decomp_p)[i][j][0]=the_points[triangle_id-1][0];
		  (*tricd_decomp_p)[i][j][1]=the_points[triangle_id-1][1];
		  (*tricd_decomp_p)[i][j][2]=the_points[triangle_id-1][2];
		}
	    }
	}
      clean_up_list_of_triangles(nr_triangles);
    }
  return ok;
}

static void kcd_tri_add_triangle(int vnr_1,int vnr_2,int vnr_3, 
		      int *nr_triangles)
{
  int new_tri = *nr_triangles;

  (*nr_triangles)++;
  list_of_triangles = MY_REALLOC(list_of_triangles,output_triangle,(*nr_triangles)-1,(*nr_triangles));
  list_of_triangles[new_tri][0] = kcd_tri_vertices[vnr_1].p3d_ref;
  list_of_triangles[new_tri][1] = kcd_tri_vertices[vnr_2].p3d_ref;
  list_of_triangles[new_tri][2] = kcd_tri_vertices[vnr_3].p3d_ref;
}

static void clip_ear(int vnr,int *nr_triangles)
{
  int i;

  /* make a triangle */
  kcd_tri_add_triangle(MOD(vnr-1,nr_vertices),vnr,MOD(vnr+1,nr_vertices),nr_triangles);
  /* eliminate vertex "vnr" from the polygon */ 
  for(i=vnr+1;i<nr_vertices;i++)
    {
      kcd_tri_vertices[i-1].ref = kcd_tri_vertices[i].ref;
      kcd_tri_vertices[i-1].p3d_ref = kcd_tri_vertices[i].p3d_ref;
      kcd_tri_vertices[i-1].x = kcd_tri_vertices[i].x;
      kcd_tri_vertices[i-1].y = kcd_tri_vertices[i].y;
    }
  nr_vertices--;
}

/* triangulates facet "facet_nr" of polyhedron "m3d_polyh" in "*nr_triangles"
   triangles listed afterwards is "*triangles" */
int kcd_tri_angulate_simple_polygon(kcd_vector3 *the_points, int *facet_i_pts, int nb_points, int *nr_triangles)
{
  int nof_points = nb_points;
  int point_it = 0;
  int poly_is_ccw = 0;

  list_of_triangles=NULL;
  (*nr_triangles) = 0;

  poly_is_ccw = kcd_tri_project_facet(the_points, facet_i_pts, nof_points);

  if(poly_is_ccw != 0)
    {
      while(nof_points > 3)
	{
	  while(!diagonal(MOD(point_it,nof_points),MOD(point_it+2,nof_points),poly_is_ccw))
	    {
	      ++point_it;
	    }
	  clip_ear(MOD(point_it+1,nof_points),nr_triangles);
	  --nof_points;
	  point_it=0;
	}
      /* and the last triangle is... */
      kcd_tri_add_triangle(MOD(point_it,nof_points),MOD(point_it+1,nof_points),
			   MOD(point_it+2,nof_points),nr_triangles);

      /* clean up */
      kcd_tri_clean();
      return TRUE;
    }
  else
    {
      return FALSE;
    }
 }

