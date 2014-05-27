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
#include "Collision-pkg.h"

/* ********************************************************************************** *
 * Function: enclose_set_by_aabb()
 *           computes AABB around set of points in their absolute position
 * ARGS IN : lofpoints    array of points, expressed w.r.t. the local coordinate frame 
 *                        of the polyhedron they belong to
 *           nb_vertices  the number of elements in the list of points lofpoints
 *           placement    placement matrix of the polyhedron w.r.t. the coordinate 
 *                        frame of the world 
 * ARGS OUT: xmin,xmax,ymin,ymax,zmin,zmax the extreme X,Y,Z coordinates of the AABB
 * ********************************************************************************** */
void enclose_set_by_aabb(kcd_vector3 *lofpoints,int nb_vertices,kcd_matrix4 *placement,
			 double *xmin, double *xmax, double *ymin, double *ymax, double *zmin, double *zmax)
{
  int j; 
  kcd_vector3 pointplaced;

  if(lofpoints[0])     
    {
      /* init extreme AABB values by first vertex: */  
      /* put vertex on its place in workspace */
      kcd_TransfoPoint(*placement,lofpoints[0],pointplaced);
      /* initialize extreme AABB values */
      *xmin = pointplaced[0];
      *xmax = pointplaced[0];
      *ymin = pointplaced[1];
      *ymax = pointplaced[1];
      *zmin = pointplaced[2];
      *zmax = pointplaced[2];
      /* visit all other vertices: */
      for(j=1;j<nb_vertices;j++)
	{
	  /* get next vertex */
	  /* put vertex on its place in workspace */
	  kcd_TransfoPoint(*placement,lofpoints[j],pointplaced);
	  /* update extreme AABB values */
	  if(*xmin > pointplaced[0])
	    *xmin = pointplaced[0];
	  if(*xmax < pointplaced[0])
	    *xmax = pointplaced[0];
	  if(*ymin > pointplaced[1])
	    *ymin = pointplaced[1];
	  if(*ymax < pointplaced[1])
	    *ymax = pointplaced[1];
	  if(*zmin > pointplaced[2])
	    *zmin = pointplaced[2];
	  if(*zmax < pointplaced[2])
	    *zmax = pointplaced[2];
	}
    }
  else
    {
      *xmin = 0.0;
      *xmax = 0.0;
      *ymin = 0.0;
      *ymax = 0.0;
      *zmin = 0.0;
      *zmax = 0.0;
    }
}

/* ********************************************************************************** *
 * Function: enclose_cylinder_by_aabb()
 *           computes AABB around cylinder in its absolute position
 * ARGS IN : r1  radius of the cylinder
 *           h   half height of the cylinder
 * ARGS OUT: xmin,xmax,ymin,ymax,zmin,zmax the extreme X,Y,Z coordinates of the AABB
 * ********************************************************************************** */
void enclose_cylinder_by_aabb(double r1,double h, 
				      kcd_matrix4 *placement,
				      double *xmin,double *xmax,double *ymin, 
				      double *ymax,double *zmin,double *zmax)
{
  double corX,corY,corZ;
  double m0,m1,m2,M0,M1,M2;


  /*  placement * (0,0,1) */
  M0 = ABS((*placement)[0][2]);
  M1 = ABS((*placement)[1][2]);
  M2 = ABS((*placement)[2][2]);
  /*  placement * (0,0,-1) */
  m0 = -M0;
  m1 = -M1;
  m2 = -M2;

  corX = ( M1 + M2 ) * r1;
  corY = ( M0 + M2 ) * r1;
  corZ = ( M0 + M1 ) * r1;

  *xmin = h * m0  - corX + (*placement)[0][3];
  *xmax = h * M0  + corX + (*placement)[0][3];
  *ymin = h * m1  - corY + (*placement)[1][3];
  *ymax = h * M1  + corY + (*placement)[1][3];
  *zmin = h * m2  - corZ + (*placement)[2][3];
  *zmax = h * M2  + corZ + (*placement)[2][3];
}

/* ********************************************************************************** *
 * Function: enclose_cone_by_aabb()
 *           computes AABB around cone in its absolute position
 * ARGS IN : r1  small radius of the cone (not used in the function)
 *           r2  large radius of the cone
 *           h   half height of the cone
 * ARGS OUT: xmin,xmax,ymin,ymax,zmin,zmax the extreme X,Y,Z coordinates of the AABB
 * PRECOND : r1 < r2 (or equal)
 * ********************************************************************************** */
void enclose_cone_by_aabb(double r1,double r2,double h, 
				      kcd_matrix4 *placement,
				      double *xmin,double *xmax,double *ymin, 
				      double *ymax,double *zmin,double *zmax)
{
  double corX,corY,corZ;
  double m0,m1,m2,M0,M1,M2;

  /*  placement * (0,0,1) */
  M0 = ABS((*placement)[0][2]);
  M1 = ABS((*placement)[1][2]);
  M2 = ABS((*placement)[2][2]);
  /*  placement * (0,0,-1) */
  m0 = -M0;
  m1 = -M1;
  m2 = -M2;

  corX = ( M1 + M2 ) * r2;
  corY = ( M0 + M2 ) * r2;
  corZ = ( M0 + M1 ) * r2;

  *xmin = h * m0  - corX + (*placement)[0][3];
  *xmax = h * M0  + corX + (*placement)[0][3];
  *ymin = h * m1  - corY + (*placement)[1][3];
  *ymax = h * M1  + corY + (*placement)[1][3];
  *zmin = h * m2  - corZ + (*placement)[2][3];
  *zmax = h * M2  + corZ + (*placement)[2][3];
}
