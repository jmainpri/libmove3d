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
#ifdef VCOLLIDE_ACT

#define VC_NON_ACTIVE -999999


/* DEBUT Modification Thibaut */
#define sqr(x) ((x) * (x))
/* FIN Modification Thibaut */

/* Global Variable for VCOLLIDE */
void *vc_hand;					/* the VCOLLIDE collision detection engine */
VCReportType *vc_colrep;		/* the VCOLLIDE collision report */

typedef int p3d_v_collide_cell;

/* global variables for VCOLLIDE calls */
static int num_vc_objects = 0;
static p3d_v_collide_cell *p3d_v_collide_array = NULL; 
static p3d_v_collide_cell *hash_on_array = NULL;
static int V_COLLIDE_INITIALIZED = 0;

#endif

/* integration of VCOLLIDE */
void p3d_v_collide_set_null()
{
#ifdef VCOLLIDE_ACT
  V_COLLIDE_INITIALIZED = 0;
#endif
}

/************************************************************************/
/* Static Functions                                                     */
/************************************************************************/
#ifdef VCOLLIDE_ACT

static int robot_to_which_body_belongs(p3d_obj *p)
{
  int nrobs = p3d_get_desc_number(P3D_ROBOT);
  int i,j,not_found=TRUE,nbods;
  p3d_obj *bod = NULL;

  for(i=0;(i<nrobs)&&(not_found);i++)
    {
      p3d_sel_desc_num(P3D_ROBOT,i);
      nbods = p3d_get_desc_number(P3D_BODY);
      for(j=0;(j<nbods)&&(not_found);j++)
	{
	  bod = (p3d_obj*)p3d_sel_desc_num(P3D_BODY,j);
	  if(p == bod)
	    not_found = FALSE;
	}
    }
  return i-1;
}

/* static double FSIGN(double f) */
/* { */
/*   double answ = 1.0; */
/*   char texte[64]; */

/*   if(EQ(f,0.0)) */
/*     { */
/*       sprintf(texte,"%f",f); */
/*       if(strncmp(texte,"-",1)==0) */
/* 	{ */
/*           //printf("f=0, answ = -1, f=%f, s=%s\n",f,texte);  */
/* 	  answ = -1.0; */
/* 	} */
/*       else */
/* 	{ */
/*         //printf("f=0, answ = 1, f=%f, s=%s\n",f,texte); */
/* 	  answ = 1.0; */
/* 	} */
/*     } */
/*   else */
/*     { */
/*       answ = SIGN(f); */
/* // printf("%f!=0, answ = %f\n",f,answ); */
/*     } */
/*   return answ; */
/* } */

static void tri_init_vc()
{
  vc_hand = NULL;
  vc_colrep = NULL;
  V_COLLIDE_INITIALIZED = 1;
}

static void tri_def_vcollide_engine()
{
  /* open vcollide engine */
  if (!vc_hand)
    vc_hand = vcOpen();
}

/* static void tri_undef_vcollide_engine() */
/* { */
/*   // close vcollide engine */ 
/*   if(vc_hand) */
/*     vcClose(vc_hand); */
/* } */

/* "input" : the scene */
/* "output": the set-up for vcollide in vc_hand (as side-effect) */
static void tri_init_vcollide()
{
  int r,nr,ir,nb;
  int nr_obst = 0,io;
  int nr_relevant_facets = 0;
  int total_nr_facets = 0;
  p3d_poly *m3d_poly_it = NULL;
  tricd_triangulation vcollide_decomp = NULL;
  tricd_vertex v1,v2,v3;
  int nr_triangles,i,vcollide_identifier,nr_robot_vc_objs = 0,
      total_nr_polyhedra,nr_polyhedra,j,nr_facet_vertices,face_num,f;
  p3d_matrix4 position_matrix;
  int non_empty_vc_obj;

  /* DEBUT Modification Thibaut */
  p3d_matrix_type scaleX,scaleY,scaleZ;
  p3d_matrix4 scaleInv,mat,position_matrix2;  
  int jj;
  /* FIN Modification Thibaut */

  /* build new vcollide engine: initialization step */
  tri_def_vcollide_engine();

  PrintInfo(("creating CC5 data structure ... triangulation\n"));
  ChronoOn();

  if(COLLISION_BY_OBJECT)
    {
      nr_obst = p3d_get_desc_number(P3D_OBSTACLE);
      nb = p3d_get_desc_number(P3D_BODIES);
      /* printf("nr_obst = %i, nb = %i, sum = %i\n",nr_obst,nb,nr_obst+nb); */
      p3d_v_collide_init_array(nr_obst+nb);
      total_nr_polyhedra = p3d_poly_get_nb();
      /* visit all obstacles in scene */
      for(io=0;io<nr_obst;io++)
	{
	  num_vc_objects++ ;
	  vcNewObject(vc_hand, &vcollide_identifier);
	  non_empty_vc_obj = FALSE;
	  /* Visit all polyhedra in obstacle */
	  p3d_sel_desc_num(P3D_OBSTACLE,io);
	  nr_polyhedra = p3d_get_obstacle_npoly();
	  /* ChronoOn(); */
	  for(j=1;j<=nr_polyhedra;j++)
	    {
	      m3d_poly_it = XYZ_ENV->ocur->pol[j-1];
	      if(m3d_poly_it->TYPE != P3D_GRAPHIC){

		/* DEBUT Modification Thibaut */
		/* we extract scale factors */
		p3d_mat4Copy(m3d_poly_it->pos_rel_jnt,mat);
		scaleX = sqrt((double) ( sqr(mat[0][0]) + sqr(mat[1][0]) + sqr(mat[2][0]) ) );
		scaleY = sqrt((double) ( sqr(mat[0][1]) + sqr(mat[1][1]) + sqr(mat[2][1]) ) );
		scaleZ = sqrt((double) ( sqr(mat[0][2]) + sqr(mat[1][2]) + sqr(mat[2][2]) ) );
		p3d_mat4Copy(p3d_mat4IDENTITY,scaleInv);
		scaleInv[0][0] = 1.0/scaleX;
		scaleInv[1][1] = 1.0/scaleY;
		scaleInv[2][2] = 1.0/scaleZ;
		/* FIN Modification Thibaut */


		face_num = poly_get_nb_faces(m3d_poly_it->poly);
		total_nr_facets += face_num;
		/* Visit all facets of this polyhedron */
		for(f = 1;f<=face_num;f++) 
		  {
		    if(p3d_filter_relevant_facet(m3d_poly_it,f,P3D_OBSTACLE))
		      {
			nr_relevant_facets++;
			/* compute number of triangles */
			nr_facet_vertices = poly_get_nb_points_in_face(m3d_poly_it->poly,f);
			nr_triangles = nr_facet_vertices - 2;
			/* allocate space for the triangles, in vcollide_decomp */
			vcollide_decomp = (tricd_triangulation)malloc(
								      sizeof(tricd_triangle)*nr_triangles);
			/* Triangulate facet */
			tri_angulate_facet(m3d_poly_it,f,nr_triangles,&vcollide_decomp);
			/* add these triangles to vcollide engine */
			for(i=0;i<nr_triangles;i++)
			  {
			    /* add a triangle */
			    non_empty_vc_obj = TRUE;
			    v1[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][0][1]
			      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[0][3];
			    v1[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][0][1]
			      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[1][3];
			    v1[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][0][1]
			      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[2][3];
			  
			    v2[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][1][1]
			      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[0][3];
			    v2[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][1][1]
			      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[1][3];
			    v2[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][1][1]
			      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[2][3];
			  
			    v3[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][2][1]
			      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[0][3];
			    v3[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][2][1]
			      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[1][3];
			    v3[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][2][1]
			      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[2][3];
			    vcAddTri(vc_hand, v1, v2, v3);
			  }
			/* created objects are active for collision by default ! */
			/* clean-up: free allocated space for vcollide_decomp */
			if(vcollide_decomp)
			  {
			    free(vcollide_decomp);
			  }
			vcollide_decomp = NULL;
		      }
		  }/* next facet */
	      }
	    }/* next polyhedron */
	  vcEndObject(vc_hand);
	  if(!non_empty_vc_obj)
	    {
	      /* the object is empty: we remove it again */
	      vcDeleteObject(vc_hand,vcollide_identifier);
	      num_vc_objects-- ;
	    }
	  else
	    {	      
/*               position is ok, don't do any vcUpdate for objects that do not move */
	      /* position_matrix = poly->pos_rel_jnt * inverse(poly->pos) */
	      /* Pos_Rel_Jnt = XYZ_ENV->ocur->pol[0]->pos_rel_jnt; */
/* 	      p3d_matInvertXform(XYZ_ENV->ocur->pol[0]->poly->pos,InvPos); */
/* 	      p3d_mat4Mult(XYZ_ENV->ocur->pol[0]->pos_rel_jnt, InvPos, position_matrix); */
/* 	      vcUpdateTrans (vc_hand, vcollide_identifier, position_matrix); */
/* 	      vcUpdateTrans (vc_hand, vcollide_identifier, p3d_mat4IDENTITY); */
	      /* printf("tri_init : obstacles : on ajoute %s numero %d avec l'id %d\n",m3d_poly_it->poly->name,m3d_poly_it->id,vcollide_identifier); */
	      p3d_v_collide_object_add_id_to_array(XYZ_ENV->ocur, vcollide_identifier);
	    }
	}/* next obstacle */

      /* visit all bodies in the scene */
      r = p3d_get_desc_curnum(P3D_ROBOT);
      nr= p3d_get_desc_number(P3D_ROBOT);
      if(nr)
	{
	  for(ir=0;ir<nr;ir++)
	    {
	      p3d_sel_desc_num(P3D_ROBOT,ir);  
	      nr_obst = p3d_get_desc_number(P3D_BODY);
	      for(io=0;io<nr_obst;io++)
		{
		  /* Visit all polyhedra in body */
		  p3d_sel_desc_num(P3D_BODY,io);
		  nr_polyhedra = p3d_get_body_npoly();
		  num_vc_objects++ ; nr_robot_vc_objs++;
		  vcNewObject(vc_hand, &vcollide_identifier);
		  non_empty_vc_obj = FALSE;
		  /* ChronoOn(); */
		  for(j=1;j<=nr_polyhedra;j++)
		    {
		      m3d_poly_it = XYZ_ENV->cur_robot->ocur->pol[j-1];	  
		      if(m3d_poly_it->TYPE != P3D_GRAPHIC){

			/* DEBUT Modification Thibaut */
			/* we extract scale factors */
			p3d_mat4Copy(m3d_poly_it->pos_rel_jnt,mat);
			scaleX = sqrt((double) ( sqr(mat[0][0]) + sqr(mat[1][0]) + sqr(mat[2][0]) ) );
			scaleY = sqrt((double) ( sqr(mat[0][1]) + sqr(mat[1][1]) + sqr(mat[2][1]) ) );
			scaleZ = sqrt((double) ( sqr(mat[0][2]) + sqr(mat[1][2]) + sqr(mat[2][2]) ) );
			p3d_mat4Copy(p3d_mat4IDENTITY,scaleInv);
			scaleInv[0][0] = 1.0/scaleX;
			scaleInv[1][1] = 1.0/scaleY;
			scaleInv[2][2] = 1.0/scaleZ;
			/* FIN Modification Thibaut */


			face_num = poly_get_nb_faces(m3d_poly_it->poly);
			total_nr_facets += face_num;
			/* Visit all facets of this polyhedron */
			for(f = 1;f<=face_num;f++)
			  {
			    if(p3d_filter_relevant_facet(m3d_poly_it,f,P3D_BODY))
			      {
				nr_relevant_facets++;
				/* compute number of triangles */
				nr_facet_vertices = poly_get_nb_points_in_face(m3d_poly_it->poly,f);
				nr_triangles = nr_facet_vertices - 2;
				/* allocate space for the triangles, in vcollide_decomp */
				vcollide_decomp = (tricd_triangulation)malloc(sizeof(tricd_triangle)*nr_triangles);
				/* Triangulate facet */
				tri_angulate_facet(m3d_poly_it,f,nr_triangles,&vcollide_decomp);
				/* add these triangles to vcollide engine */
				for(i=0;i<nr_triangles;i++)
				  {
				    /* add a triangle */
				    non_empty_vc_obj = TRUE;
				    v1[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][0][1]
				      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[0][3];
				    v1[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][0][1]
				      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[1][3];
				    v1[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][0][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][0][1]
				      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][0][2] + m3d_poly_it->pos_rel_jnt[2][3];
				    
				    v2[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][1][1]
				      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[0][3];
				    v2[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][1][1]
				      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[1][3];
				    v2[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][1][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][1][1]
				      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][1][2] + m3d_poly_it->pos_rel_jnt[2][3];

				    v3[0] = m3d_poly_it->pos_rel_jnt[0][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[0][1] * vcollide_decomp[i][2][1]
				      + m3d_poly_it->pos_rel_jnt[0][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[0][3];
				    v3[1] = m3d_poly_it->pos_rel_jnt[1][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[1][1] * vcollide_decomp[i][2][1]
				      + m3d_poly_it->pos_rel_jnt[1][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[1][3];
				    v3[2] = m3d_poly_it->pos_rel_jnt[2][0] * vcollide_decomp[i][2][0] + m3d_poly_it->pos_rel_jnt[2][1] * vcollide_decomp[i][2][1]
				      + m3d_poly_it->pos_rel_jnt[2][2] * vcollide_decomp[i][2][2] + m3d_poly_it->pos_rel_jnt[2][3];

				    vcAddTri(vc_hand, v1, v2, v3);
				  }
				/* created objects are active for collision by default ! */
				/* clean-up: free allocated space for vcollide_decomp */
				if(vcollide_decomp)
				  {
				    free(vcollide_decomp);
				  }
				vcollide_decomp = NULL;
			      }
			  }/* next facet */
		      }
		    }
		  vcEndObject(vc_hand);
		  if(!non_empty_vc_obj)
		    {
		      /* the object is empty: we remove it again */
		      vcDeleteObject(vc_hand,vcollide_identifier);
		      num_vc_objects-- ; nr_robot_vc_objs-- ;
		    }
		  else
		    {
		      vcUpdateTrans (vc_hand, vcollide_identifier, XYZ_ENV->cur_robot->ocur->jnt->pos );
		      p3d_v_collide_object_add_id_to_array(XYZ_ENV->cur_robot->ocur, vcollide_identifier);
		    }
		}/* next body */
	    } /* next robot */
	  p3d_sel_desc_num(P3D_ROBOT,r);
	}
      /* end COLLISION BY OBJECT */
    }
  else
    {
      total_nr_polyhedra = p3d_poly_get_nb();
      p3d_v_collide_init_array(total_nr_polyhedra);
    
      /* visit all obstacles in scene */
      nr_obst = p3d_get_desc_number(P3D_OBSTACLE);
      for(io=0;io<nr_obst;io++)
	{
	  /* Visit all polyhedra in obstacle */
	  p3d_sel_desc_num(P3D_OBSTACLE,io);
	  nr_polyhedra = p3d_get_obstacle_npoly();
	  /* ChronoOn(); */
	  for(j=1;j<=nr_polyhedra;j++)
	    {
	      m3d_poly_it = XYZ_ENV->ocur->pol[j-1];
	      if(m3d_poly_it->TYPE != P3D_GRAPHIC){
		non_empty_vc_obj = FALSE;
		num_vc_objects++ ;
		vcNewObject(vc_hand, &vcollide_identifier);
		/* printf("triangulating polyhedron %s ...\n",m3d_poly_it->poly->name); */

		/* DEBUT Modification Thibaut */
		/* we extract scale factors */
		p3d_mat4Copy(m3d_poly_it->pos_rel_jnt,mat);
		scaleX = sqrt((double) ( sqr(mat[0][0]) + sqr(mat[1][0]) + sqr(mat[2][0]) ) );
		scaleY = sqrt((double) ( sqr(mat[0][1]) + sqr(mat[1][1]) + sqr(mat[2][1]) ) );
		scaleZ = sqrt((double) ( sqr(mat[0][2]) + sqr(mat[1][2]) + sqr(mat[2][2]) ) );
		p3d_mat4Copy(p3d_mat4IDENTITY,scaleInv);
		scaleInv[0][0] = 1.0/scaleX;
		scaleInv[1][1] = 1.0/scaleY;
		scaleInv[2][2] = 1.0/scaleZ;
		/* FIN Modification Thibaut */

		face_num = poly_get_nb_faces(m3d_poly_it->poly);
		total_nr_facets += face_num;
		/* Visit all facets of this polyhedron */
		for(f = 1;f<=face_num;f++) 
		  {
		    if(p3d_filter_relevant_facet(m3d_poly_it,f,P3D_OBSTACLE))
		      {
			nr_relevant_facets++;
			/* compute number of triangles */
			nr_facet_vertices = poly_get_nb_points_in_face(m3d_poly_it->poly,f);
			nr_triangles = nr_facet_vertices - 2;
			/* allocate space for the triangles, in vcollide_decomp */
			vcollide_decomp = (tricd_triangulation)malloc(
								      sizeof(tricd_triangle)*nr_triangles);
			/* Triangulate facet */
			tri_angulate_facet(m3d_poly_it,f,nr_triangles,&vcollide_decomp);
			/* add these triangles to vcollide engine */
			for(i=0;i<nr_triangles;i++)
			  {
			    /* add a triangle */
			    non_empty_vc_obj = TRUE;
			    /* DEBUT Modification Thibaut */
			    for(jj=0;jj<3;jj++) {
			      vcollide_decomp[i][jj][0] *= scaleX;
			      vcollide_decomp[i][jj][1] *= scaleY;
			      vcollide_decomp[i][jj][2] *= scaleZ;
			    }
			    /* FIN Modification Thibaut */
			    vcAddTri(vc_hand, vcollide_decomp[i][0],
				     vcollide_decomp[i][1],vcollide_decomp[i][2]);
			  }
			/* created objects are active for collision by default ! */
			/* clean-up: free allocated space for vcollide_decomp */
			if(vcollide_decomp)
			  {
			    free(vcollide_decomp);
			  }
			vcollide_decomp = NULL;
		      }
		  }/* next facet */
		vcEndObject(vc_hand);
		if(!non_empty_vc_obj)
		  {
		    /* the object is empty: we remove it again */
		    vcDeleteObject(vc_hand,vcollide_identifier);
		    num_vc_objects-- ;
		  }
		else
		  {
		    poly_get_poly_pos(m3d_poly_it->poly, position_matrix);
		    /* DEBUT Modification Thibaut */	      
		    p3d_mat4Mult(position_matrix,scaleInv,position_matrix2);
		    p3d_mat4Copy(position_matrix2,position_matrix);
		    /* FIN Modification Thibaut */
		    vcUpdateTrans (vc_hand, vcollide_identifier, position_matrix);
		    p3d_v_collide_add_id_to_array(m3d_poly_it, vcollide_identifier);
		  }
	      }
	    }/* next polyhedron */
	}/* next obstacle */
      
      /* visit all bodies in the scene */
      r = p3d_get_desc_curnum(P3D_ROBOT);
      nr= p3d_get_desc_number(P3D_ROBOT);
      if(nr) 
	{
	  for(ir=0;ir<nr;ir++) 
	    {
	      p3d_sel_desc_num(P3D_ROBOT,ir);  
	      nr_obst = p3d_get_desc_number(P3D_BODY);
	      for(io=0;io<nr_obst;io++)
		{
		  /* Visit all polyhedra in body */
		  p3d_sel_desc_num(P3D_BODY,io);
		  nr_polyhedra = p3d_get_body_npoly();
		  /* ChronoOn(); */
		  for(j=1;j<=nr_polyhedra;j++)
		    {
		      m3d_poly_it = XYZ_ENV->cur_robot->ocur->pol[j-1];	  
		      if(m3d_poly_it->TYPE != P3D_GRAPHIC){
			non_empty_vc_obj = FALSE;
			num_vc_objects++ ; nr_robot_vc_objs++;
			vcNewObject(vc_hand, &vcollide_identifier);
			/* printf("triangulating polyhedron %s ...\n",m3d_poly_it->poly->name); */
			/* DEBUT Modification Thibaut */
			/* we extract scale factors */
			p3d_mat4Copy(m3d_poly_it->pos_rel_jnt,mat);
			scaleX = sqrt((double) ( sqr(mat[0][0]) + sqr(mat[1][0]) + sqr(mat[2][0]) ) );
			scaleY = sqrt((double) ( sqr(mat[0][1]) + sqr(mat[1][1]) + sqr(mat[2][1]) ) );
			scaleZ = sqrt((double) ( sqr(mat[0][2]) + sqr(mat[1][2]) + sqr(mat[2][2]) ) );
			p3d_mat4Copy(p3d_mat4IDENTITY,scaleInv);
			scaleInv[0][0] = 1.0/scaleX;
			scaleInv[1][1] = 1.0/scaleY;
			scaleInv[2][2] = 1.0/scaleZ;
			/* FIN Modification Thibaut */


			face_num = poly_get_nb_faces(m3d_poly_it->poly);
			total_nr_facets += face_num;
			/* Visit all facets of this polyhedron */
			for(f = 1;f<=face_num;f++) 
			  {
			    if(p3d_filter_relevant_facet(m3d_poly_it,f,P3D_BODY))
			      {
				nr_relevant_facets++;
				/* compute number of triangles */
				nr_facet_vertices = poly_get_nb_points_in_face(m3d_poly_it->poly,f);
				nr_triangles = nr_facet_vertices - 2;
				/* allocate space for the triangles, in vcollide_decomp */
				vcollide_decomp = (tricd_triangulation)malloc(
									      sizeof(tricd_triangle)*nr_triangles);
				/* Triangulate facet */
				tri_angulate_facet(m3d_poly_it,f,nr_triangles,&vcollide_decomp);
				/* add these triangles to vcollide engine */
				for(i=0;i<nr_triangles;i++)
				  {
				/* add a triangle */
				    non_empty_vc_obj = TRUE;
				    /* DEBUT Modification Thibaut */
				    for(jj=0;jj<3;jj++) {
				      vcollide_decomp[i][jj][0] *= scaleX;
				      vcollide_decomp[i][jj][1] *= scaleY;
				      vcollide_decomp[i][jj][2] *= scaleZ;
				    }
				    /* FIN Modification Thibaut */
				    vcAddTri(vc_hand, vcollide_decomp[i][0],
					     vcollide_decomp[i][1],vcollide_decomp[i][2]);
				  }
				/* created objects are active for collision by default ! */
				/* clean-up: free allocated space for vcollide_decomp */
				if(vcollide_decomp)
				  {
				    free(vcollide_decomp);
				  }
				vcollide_decomp = NULL;
			      }
			  }/* next facet */
			vcEndObject(vc_hand);
			if(!non_empty_vc_obj)
			  {
			    /* the object is empty: we remove it again */
			    vcDeleteObject(vc_hand,vcollide_identifier);
			    num_vc_objects-- ; nr_robot_vc_objs-- ;
			  }
			else
			  {
			    poly_get_poly_pos(m3d_poly_it->poly, position_matrix);
			    /* DEBUT Modification Thibaut */	      
			    p3d_mat4Mult(position_matrix,scaleInv,position_matrix2);
			    p3d_mat4Copy(position_matrix2,position_matrix); 
			    /* FIN Modification Thibaut */
			    vcUpdateTrans (vc_hand, vcollide_identifier, position_matrix);
			    p3d_v_collide_add_id_to_array(m3d_poly_it, vcollide_identifier);
			  }
		      }
		    }
		}/* next body */
	    } /* next robot */
	  p3d_sel_desc_num(P3D_ROBOT,r);
	}
    }

  /* activate pairs (exactly the way in which it is done for I_COLLIDE) */
  PrintInfo(("Triangulation done.\n"));
  ChronoPrint("filter");
  ChronoOff();
  PrintInfo(("nr of facets = %i, nr_activated_facets = %i, nr_vc_objects = %i (of robot:%i)\n",
	     total_nr_facets,nr_relevant_facets,num_vc_objects,nr_robot_vc_objs));
}


#endif
/************************************************************************/
/* Non Static Functions                                                     */
/************************************************************************/

void p3d_v_collide_init_array(int sz)
{
#ifdef VCOLLIDE_ACT
  int i, nobs = p3d_get_desc_number(P3D_OBSTACLE), nrobs = p3d_get_desc_number(P3D_ROBOT);

  if(p3d_v_collide_array)
    free(p3d_v_collide_array);
  p3d_v_collide_array=NULL;
  p3d_v_collide_array=(p3d_v_collide_cell *)malloc(sizeof(p3d_v_collide_cell)*sz);
  for(i=0;i<sz;i++)
    {
      p3d_v_collide_array[i] = VC_NON_ACTIVE;
    }
  if(hash_on_array)
    free(hash_on_array);
  hash_on_array = NULL;
  hash_on_array = (p3d_v_collide_cell *)malloc(sizeof(p3d_v_collide_cell)*nrobs);
  hash_on_array[0] = nobs;
  for(i=1;i<nrobs;i++)
    {
      hash_on_array[i]= hash_on_array[i-1] + XYZ_ENV->robot[i-1]->no;
    }
#endif
}

int p3d_v_collide_is_non_active(p3d_poly *obj)
{
#ifdef VCOLLIDE_ACT
  return(p3d_v_collide_get_id(obj)==VC_NON_ACTIVE); 
#else
  return 0;
#endif
 
}

int p3d_v_collide_object_is_non_active(p3d_obj *obj)
{
#ifdef VCOLLIDE_ACT
  return(p3d_v_collide_object_get_id(obj)==VC_NON_ACTIVE); 
#else
  return 0;
#endif
}

void p3d_v_collide_add_id_to_array(p3d_poly *p, int v_collide_id)
{
#ifdef VCOLLIDE_ACT
  if(p==NULL)
    {
      PrintInfo(("ERROR in p3d_v_collide_add_array_poly_to_id: no such polyhedron \n"));
    }
  else
    {
      p3d_v_collide_array[p->id]=v_collide_id;
    }
#endif
}



void p3d_v_collide_object_add_id_to_array(p3d_obj *p, int v_collide_id)
{
#ifdef VCOLLIDE_ACT
  int nr_obst=0,currob;
  if(p==NULL)
    {
      PrintInfo(("ERROR in p3d_v_collide_object_add_array_poly_to_id: no such object \n"));
    }
  else
    {
      if(p->type == P3D_OBSTACLE)
	p3d_v_collide_array[p->num]=v_collide_id;
      else
	{
	  
	  currob = robot_to_which_body_belongs(p);
	  nr_obst = hash_on_array[currob];
	  p3d_v_collide_array[p->num + nr_obst]=v_collide_id;
	}
    }
#endif
}

int p3d_v_collide_get_id(p3d_poly *p)
{ 
#ifdef VCOLLIDE_ACT
  return p3d_v_collide_array[p->id];
#else
  return 0;
#endif
}

int p3d_v_collide_object_get_id(p3d_obj *obj)
{ 
#ifdef VCOLLIDE_ACT
  int nr_obst=0,currob;

  if(obj->type == P3D_OBSTACLE)
    {
      return p3d_v_collide_array[obj->num];
    }
  else
    {
      currob = robot_to_which_body_belongs(obj);
      nr_obst = hash_on_array[currob];
      return p3d_v_collide_array[obj->num + nr_obst];
    }
#else
  return 0;
#endif
}

p3d_poly* p3d_v_collide_get_poly_by_id(int id)
{ 
#ifdef VCOLLIDE_ACT
  p3d_poly *p;

  p=p3d_poly_get_first();
  while((p!=NULL) && (p3d_v_collide_get_id(p)!=id))
    p=p3d_poly_get_next();
  if (p==NULL) PrintInfo(("\nErreur id non valide dans p3d_v_collide_get_poly_by_id:%i\n",id));
  return(p);
#else
  return NULL;
#endif
}

p3d_obj* p3d_v_collide_get_object_by_id(int id)
{ 
#ifdef VCOLLIDE_ACT
  p3d_obj *p;
  int i,j,no,nb,not_found = TRUE;
  
  no = p3d_get_desc_number(P3D_OBSTACLE);
  /* OBSTACLES */
  for(i=0;(i<no) && (not_found);i++)
    {
      p=XYZ_ENV->o[i];
      not_found = (p3d_v_collide_object_get_id(p)!=id );
    }
  /* ROBOT BODIES */
  no = p3d_get_desc_number(P3D_ROBOT);
  for(i=0;(i<no) && (not_found);i++)
    {
      nb = XYZ_ENV->nr;
      for(j=0;(j<nb) && (not_found);j++)
	{
	  p=XYZ_ENV->robot[j]->o[i];
	  not_found = (p3d_v_collide_object_get_id(p)!=id );
	}
    }

  if (not_found ) PrintInfo(("\nErreur id non valide dans p3d_v_collide_get_object_by_id:%i\n",id));
  return(p);
#else
  return NULL;
#endif
}

void p3d_v_collide_set_pos_of_object(p3d_obj *p, p3d_matrix4 mat)
{
#ifdef VCOLLIDE_ACT
  int id = -999999;

/* DEBUT Modification Thibaut */
p3d_matrix_type scaleX,scaleY,scaleZ;
p3d_matrix4 scaleInv,mat2;  
  /* we extract scale factors */
  scaleX = sqrt((double) ( sqr(mat[0][0]) + sqr(mat[1][0]) + sqr(mat[2][0]) ) );
  scaleY = sqrt((double) ( sqr(mat[0][1]) + sqr(mat[1][1]) + sqr(mat[2][1]) ) );
  scaleZ = sqrt((double) ( sqr(mat[0][2]) + sqr(mat[1][2]) + sqr(mat[2][2]) ) );

  /* we remove them */
  p3d_mat4Copy(p3d_mat4IDENTITY,scaleInv);
  scaleInv[0][0] = 1.0/scaleX;
  scaleInv[1][1] = 1.0/scaleY;
  scaleInv[2][2] = 1.0/scaleZ;
  p3d_mat4Mult(mat,scaleInv,mat2);
  p3d_mat4Copy(mat2,mat);
/* FIN Modification Thibaut */

  id = p3d_v_collide_object_get_id(p);
  if(id != -999999)
    vcUpdateTrans (vc_hand, id, mat);
#endif
}

void p3d_v_collide_set_pos(p3d_poly *p, p3d_matrix4 mat)
{
#ifdef VCOLLIDE_ACT
  int id = 0;

  id = p3d_v_collide_get_id(p);
  vcUpdateTrans (vc_hand, id, mat);
#endif
}

int p3d_v_collide_get_num_vc_objects()
{
#ifdef VCOLLIDE_ACT
  return num_vc_objects;
#else
  return 0;
#endif
}

void p3d_v_collide_start()
{
#ifdef VCOLLIDE_ACT
  /* initialize filter */
  if(p3d_filter_needs_init())
    p3d_filter_init_filter();
  /* first time ever V_COLLIDE is started */
  if (V_COLLIDE_INITIALIZED == 0)
    tri_init_vc();
  /* we must prepare data for V_COLLIDE */
  if(vc_hand == NULL)
    tri_init_vcollide();
#endif
}

void p3d_v_collide_del_poly(p3d_poly *p)
{
#ifdef VCOLLIDE_ACT
  int id=0;

  if (V_COLLIDE_INITIALIZED != 0)
    {
      id = p3d_v_collide_get_id(p);
      vcDeleteObject(vc_hand,id);
    }
#endif
}

void p3d_v_collide_stop()
{
#ifdef VCOLLIDE_ACT
  PrintInfo(("Destruction structure CC5\n"));

  if (V_COLLIDE_INITIALIZED != 0){
    if(vc_colrep)
      free(vc_colrep);
    vcClose (vc_hand);
    V_COLLIDE_INITIALIZED = 0;
  }
  if(p3d_v_collide_array)
    free(p3d_v_collide_array);
  p3d_v_collide_array = NULL;
  if(hash_on_array)
    free(hash_on_array);
  hash_on_array = NULL;
  PrintInfo(("Destruction structure CC5 done\n"));
#endif
}

