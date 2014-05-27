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
#ifdef GJK_DEBUG

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"


/* Modifications Pepijn may  2001
 * GOAL : slow but exact distance computation
 * 
 * This will be obtained by replacing the function kvd_gjk_just_intersect by
 * the function kcd_gjk_closest_pair
 *
 * The results will be a report containing the position of the closest points 
 * and the minimum distance (for each body of each robot)
 *
 *
 */

/* IMPLEMENTATION NOTES (may 2001)
 * 
 * The initialisation of zpa and zpb is important 
 * zpa et zpb replace v
 */



static double gjk_debug_tolerance = 0.0;
void gjk_debug_set_tolerance(double value)
{
  gjk_debug_tolerance = value;
}
void gjk_debug_get_tolerance(double *value)
{ 
  *value = gjk_debug_tolerance;
}


/* Just a function for debugging GJK */
int p3d_gjk_collision_test()
{
  int polyh1_entity_type,polyh2_entity_type;
  int collision_exists = FALSE;
  int collision_exists1 = FALSE;
  int collision_exists2 = FALSE;
  int i,j,k,nof_bodies,tot_nof_polys=0,tot_nof_robot_polys=0,nof_obst,nr_robots,nof_polys_r,nof_polys_o,pit,qit,u,uu,w,n_facets1,n_facets;
  int nof_vertices1=0,obst_faces_so_far;
  int *facet_ids1=NULL;
  int kcd_gjk_id1;
  void *polyh1=NULL;
  int nof_vertices2=0;
  int *facet_ids2=NULL;
  int kcd_gjk_id2;
  void *polyh2=NULL;
  kcd_matrix4 *AwrtW, *BwrtW, *AwrtBODY;
  kcd_vector3 zpa, zpb;
  double gjk_tolerance = gjk_debug_tolerance;

  /* minimum distance between two points */ 
  double  min_distance = P3D_HUGE; 
  /* smallest distance for a certain body of a certain robot*/
  double smallest_min_distance = P3D_HUGE; 
  /* corresponding primitive of the body for the closest point 
   * used to calculate the correct relative position
   * (relative to the center of the body) 
   */
  int closest_primitive = 0;
  /* corresponding information to retrieve matrix BwrtW for the closest point 
   * this matrix is used to calculate the correct absolute coordinate
   * of point B
   */ 
  int closest_obj=0, closest_prim_obj=0;
   /* keep also the les zpi's */
   kcd_vector3 closest_zpa,closest_zpb;


  /* P: gjk_tolerance
   * The default value is 0.0 (for exact distance computation)
   * This tolerance is compared with a certaind distance in kcd_gjk_closest_pair
   */


  kcd_set_report_to_zero();
  nr_robots = XYZ_ENV->nr;
  nof_obst = XYZ_ENV->no;
  for(i=0;i<nr_robots;i++)
    for(j=0;j<XYZ_ENV->robot[i]->no;j++)
      for(u=0;u<XYZ_ENV->robot[i]->o[j]->np;u++)
	if((XYZ_ENV->robot[i]->o[j]->pol[u]->entity_type == CONVEX_POLYHEDRON) || 
	   (XYZ_ENV->robot[i]->o[j]->pol[u]->entity_type == POLYHEDRON_ENTITY))
	  tot_nof_polys += XYZ_ENV->robot[i]->o[j]->pol[u]->poly->nb_faces;
	else
	  tot_nof_polys += 1;
  tot_nof_robot_polys = tot_nof_polys;
   for(i=0;i<nof_obst;i++)
    for(u=0;u<XYZ_ENV->o[i]->np;u++)
	if((XYZ_ENV->o[i]->pol[u]->entity_type == CONVEX_POLYHEDRON) || 
	   (XYZ_ENV->o[i]->pol[u]->entity_type == POLYHEDRON_ENTITY))
	  tot_nof_polys += XYZ_ENV->o[i]->pol[u]->poly->nb_faces;
	else
	  tot_nof_polys += 1;

  kcd_gjk_support_initialize_interesting_seed(tot_nof_polys+1);
  kcd_gjk_id1 = tot_nof_polys;
  kcd_gjk_id2 = tot_nof_polys;

  /* memory allocation 1 for the report mecanism
   * FUTURE AMELIORATION:
   * do both memory allocations in the initialisation phase 
   */
  kcd_init_kcd_distance_report_table(nr_robots);


  /* visit all robots */
  for(i=0;i<nr_robots;i++)
    {
      nof_bodies = XYZ_ENV->robot[i]->no;
      
      /* memory allocation 2 for the report mecanism*/
      kcd_init_kcd_distance_report_table_entry(i,nof_bodies);

      /* visit all bodies */
      for(j=0;j<nof_bodies;j++)
	{
	  /* For each body we have to reset the 
	   * smallest_min_distance for the report mecanism,
	   * for the report mecanism   
	   */
	  smallest_min_distance = P3D_HUGE;
	  zpa[0]= P3D_HUGE; zpa[1]= P3D_HUGE; zpa[2]= P3D_HUGE;
	  zpb[0]= P3D_HUGE; zpb[1]= P3D_HUGE; zpb[2]= P3D_HUGE;
	  /* end init report mecanism */

	  /* visit all primitives of the body */
	  nof_polys_r = XYZ_ENV->robot[i]->o[j]->np;
         // PrintInfo(("nof_polys %d , j = %d  nof_bodies %d\n",  nof_polys_r,j, nof_bodies ));
	  for(pit=0;pit<nof_polys_r;pit++)
	    {
	      polyh1 = XYZ_ENV->robot[i]->o[j]->pol[pit];

	      /* P:TRUE means that it can move => ROBOT 
	       * get AwrtW
	       kcd_get_prim_abs_pos(polyh1,TRUE,&polyh1_poly_pos);*/
	      kcd_get_prim_abs_pos(polyh1,TRUE,&AwrtW); 

	      /* P: this matrix doesn't need to be inverted because we need
                 AwrtW = polyh1_poly_pos
		 kcd_matInvertTransfo(*polyh1_poly_pos,mat1);
	       */ 

	      nof_vertices1 = 0;
	      facet_ids1 = NULL;
	      polyh1_entity_type = kcd_get_poly_entity_type(polyh1);
	      /* visit all obstacles */
	      obst_faces_so_far = 0;
	      for(k=0;k<nof_obst;k++)
		{
		  /* visit all primitives of the obstacle */
		  nof_polys_o = XYZ_ENV->o[k]->np;
		  for(qit=0;qit < nof_polys_o;qit++)
		    {
		      polyh2 = XYZ_ENV->o[k]->pol[qit];
		      polyh2_entity_type = kcd_get_poly_entity_type(polyh2);
		      /* PrintInfo(("kcd_gjk_id2 = %i\n",kcd_gjk_id2)); */
		      nof_vertices2 = 0;
		      facet_ids2 = NULL;
		      
		      /* do collision test */
#ifdef CP_IS_SOLID
		      if(((polyh1_entity_type == SPHERE_ENTITY) || (polyh1_entity_type == CYLINDER_ENTITY) || 
			  (polyh1_entity_type == BOX_ENTITY)    || (polyh1_entity_type == CUBE_ENTITY) || 
			  (polyh1_entity_type == CONE_ENTITY)   || (polyh1_entity_type == CONVEX_POLYHEDRON) ||
			  (polyh1_entity_type == POLYHEDRON_ENTITY) ) &&
			 ((polyh2_entity_type == SPHERE_ENTITY) || (polyh2_entity_type == CYLINDER_ENTITY) || 
			  (polyh2_entity_type == BOX_ENTITY)    || (polyh2_entity_type == CUBE_ENTITY) || 
			  (polyh2_entity_type == CONE_ENTITY)   || (polyh2_entity_type == CONVEX_POLYHEDRON) ||
			  (polyh2_entity_type == POLYHEDRON_ENTITY) ) 
			 )
#else
		      if(((polyh1_entity_type == SPHERE_ENTITY) || (polyh1_entity_type == CYLINDER_ENTITY) || 
			  (polyh1_entity_type == BOX_ENTITY)    || (polyh1_entity_type == CUBE_ENTITY) || 
			  (polyh1_entity_type == CONE_ENTITY) )               &&
			 ((polyh2_entity_type == SPHERE_ENTITY) || (polyh2_entity_type == CYLINDER_ENTITY) || 
			  (polyh2_entity_type == BOX_ENTITY)    || (polyh2_entity_type == CUBE_ENTITY) || 
			  (polyh2_entity_type == CONE_ENTITY) ) 
			 )
#endif			 
			{
			  /* PrintInfo(("big if %s vs %s\n",polyh1->poly->name,polyh2->poly->name)); */
			  if(polyh1_entity_type == POLYHEDRON_ENTITY) polyh1_entity_type = CONVEX_POLYHEDRON;
			  if(polyh2_entity_type == POLYHEDRON_ENTITY) polyh2_entity_type = CONVEX_POLYHEDRON;
	  
			  /* initialisation of the zpi's */
			  zpa[0]=0.0;zpa[1]=0.0;zpa[2]=0.0;
			  zpb[0]=0.0;zpb[1]=0.0;zpb[2]=0.0;

			  /* p3d_matInvertXform(polyh2->pos0,mat2); */
			  /* kcd_get_poly_pos0(polyh2,&polyh2_pos0); */
			  
			  /* Get BwrtW 
			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);*/
			  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);

			  /* We don't need to calculate BwrtA anymore 
			     kcd_matMultTransfo(mat1,*polyh2_pos0,BwrtA);*/
			  /* p3d_matMultXform(mat1,polyh2->pos0,BwrtA); */
			  /* p3d_mat4Print(polyh2->pos0,"obj"); */
			  /* p3d_mat4Print(BwrtA,"BwrtA"); */

			  obst_faces_so_far++;
			  kcd_gjk_set_interesting_seed(kcd_gjk_id1,0);			  

			  /* kcd_gjk_just_intersect is replaced by kcd_gjk_closest_pair
	    		  collision_exists1 =  kcd_gjk_just_intersect(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
								     kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
								     BwrtA, &v);*/
			  collision_exists1 = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
								   kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
								   *AwrtW, *BwrtW, &zpa, &zpb, &min_distance, gjk_tolerance); 

			  if(min_distance < smallest_min_distance )
			    {
			      /* save the necessary information to fill in the report*/
			      closest_obj = k;
			      closest_prim_obj = qit;
			      closest_primitive = pit;
			      kcd_vectCopy(zpa,closest_zpa);
			      kcd_vectCopy(zpb,closest_zpb);
			      smallest_min_distance =  min_distance;
			    }
			  /*PrintInfo(("Only Solids ... \n"));
			  PrintInfo(("info zpa = %lf %lf %lf  / zpb =%lf %lf %lf \n", zpa[0], zpa[1],zpa[2],  zpb[0], zpb[1],zpb[2]));   
			  PrintInfo(("info min_distance = %lf \n\n", min_distance));   */


			}
#ifdef CP_IS_SOLID
		      /* we don't treat convex polyhedrons seperately */
		      /* here concave polyhedrons are not considered */
#else
		      else if( ((polyh2_entity_type == POLYHEDRON_ENTITY)||(polyh2_entity_type == CONVEX_POLYHEDRON)) && 
			       (polyh1_entity_type != POLYHEDRON_ENTITY)&&(polyh1_entity_type != CONVEX_POLYHEDRON)&&
			       (polyh1_entity_type != CONCAVE_POLYHEDRON)  )
			{
		
			  /* Get BwrtW
			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);*/
			  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);

		
			  /* initialisation of the zpa */
		      	  zpa[0]=0.0; zpa[1]=0.0; zpa[2]=0.0;
		
			
			  /* kcd_matMultTransfo(mat1,*polyh2_pos0,BwrtA);*/
			  /* p3d_matMultXform(mat1,polyh2->pos0,BwrtA); */
			  /* p3d_mat4Print(polyh2->pos0,"obj"); */
			  /* p3d_mat4Print(BwrtA,"BwrtA"); */

			  n_facets = kcd_get_nb_fs(polyh2);
			  /* n_facets = polyh2->poly->nb_faces; */
			  for(u=0;u<n_facets;u++)
			    {
			      nof_vertices2 = kcd_get_nb_pts_in_f(polyh2,u+1);
			      /* nof_vertices2 = p3d_get_nb_points_in_face(polyh2->poly,u+1); */
			      facet_ids2 = MY_ALLOC(int,nof_vertices2);
			      for(w=0;w<nof_vertices2;w++)
				{
				  facet_ids2[w] = kcd_get_i_pt_in_f(polyh2,u+1,w+1);
				  /* facet_ids2[w] = p3d_get_index_point_in_face(polyh2->poly,u+1,w+1); */
				  /* PrintInfo(("facet %i: id %i\n",u,facet_ids2[w])); */
				}
			      kcd_gjk_set_interesting_seed(kcd_gjk_id1,0);

			      /* initialisation of the zpb */
			      kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2])); 

			      collision_exists2 = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
								   kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
								   *AwrtW, *BwrtW, &zpa, &zpb, &min_distance, gjk_tolerance); 
			 	
			     /* PrintInfo(("Solid Body <-> POLYHEDRON  ... \n"));
			      PrintInfo(("info zpa = %lf %lf %lf  / zpb =%lf %lf %lf \n", zpa[0], zpa[1],zpa[2],  zpb[0], zpb[1],zpb[2]));   
			      PrintInfo(("info min_distance = %lf \n\n", min_distance));  */
			
			      if(min_distance < smallest_min_distance )
				{			
				  /* save the necessary information to fill in the report*/
				  closest_obj = k;
				  closest_prim_obj = qit;
				  closest_primitive = pit;
				  kcd_vectCopy(zpa,closest_zpa);
				  kcd_vectCopy(zpb,closest_zpb);
				  smallest_min_distance =  min_distance;
				}
			      

			      /*collision_exists2 =  kcd_gjk_just_intersect(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
									  kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
									  BwrtA, &v);*/
			      collision_exists1 = collision_exists1 || collision_exists2;

			      MY_FREE(facet_ids2,int,nof_vertices2);
			    }
			}
		      else if( ((polyh1_entity_type == POLYHEDRON_ENTITY)||(polyh1_entity_type == CONVEX_POLYHEDRON)) && 
			       (polyh2_entity_type != POLYHEDRON_ENTITY)&&(polyh2_entity_type != CONVEX_POLYHEDRON)&&
			       (polyh2_entity_type != CONCAVE_POLYHEDRON) )
			{
/* 			  if(!p3d_poly_is_convex(polyh1->poly)) */
/* 			    { */
/* 			      PrintInfo(("polyh1 concave !\n")); */
/* 			    } */
/* 			  else */
/* 			    { */
/* 			      PrintInfo(("polyh1 is convex polyhedron\n")); */
/* 			    } */
/*			  v[0]=0.0;
			  v[1]=0.0; 
			  v[2]=0.0;*/			 
	
/* 			  kcd_get_poly_pos0(polyh2,&polyh2_pos0); */


			  /* Get BwrtW 
			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);*/
			  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);

/*			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);*/
/*			  kcd_matMultTransfo(mat1,*polyh2_pos0,BwrtA);*/
			  /* p3d_matMultXform(mat1,polyh2->pos0,BwrtA); */
			  n_facets = kcd_get_nb_fs(polyh1); 
			  /* n_facets = polyh1->poly->nb_faces; */
			  for(u=0;u<n_facets;u++)
			    {
			      nof_vertices1 = kcd_get_nb_pts_in_f(polyh1,u+1);
			      /* nof_vertices1 = p3d_get_nb_points_in_face(polyh1->poly,u+1); */
			      facet_ids1 = MY_ALLOC(int,nof_vertices1);
			      for(w=0;w<nof_vertices1;w++)
				{
				  facet_ids1[w] = kcd_get_i_pt_in_f(polyh1,u+1,w+1);
				  /* facet_ids1[w] = p3d_get_index_point_in_face(polyh1->poly,u+1,w+1); */
				}
			      kcd_gjk_set_interesting_seed(kcd_gjk_id1,0);

			      /* initialisation of the zpi's */
			      kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2])); 
			      zpb[0]=0.0; zpb[1]=0.0; zpb[2]=0.0;
			      /* end init */
			      collision_exists2 = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
								   kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
								   *AwrtW, *BwrtW, &zpa, &zpb, &min_distance, gjk_tolerance); 
			  
			      /*PrintInfo(("Polyhedron  Body <-> Solid Object  ... \n"));
			      PrintInfo(("info zpa = %lf %lf %lf  / zpb =%lf %lf %lf \n", zpa[0], zpa[1],zpa[2],  zpb[0], zpb[1],zpb[2]));  
			      PrintInfo(("info min_distance = %lf \n\n", min_distance));  */
		
			       if(min_distance < smallest_min_distance )
				 {			
				   /* save the necessary information to fill in the report*/
				   closest_obj = k;
				   closest_prim_obj = qit;
				   closest_primitive = pit;
				   kcd_vectCopy(zpa,closest_zpa);
				   kcd_vectCopy(zpb,closest_zpb);
				   smallest_min_distance =  min_distance;
				 }



/*			      collision_exists2 =  kcd_gjk_just_intersect(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
	 								  kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
		 							  BwrtA, &v);*/
			      collision_exists1 = collision_exists1 || collision_exists2; 
/* 			      if(collision_exists2) */
/* 				{ */
/* 				  PrintInfo(("baf: facet %i\n",u)); */
/* 				} */
			      MY_FREE(facet_ids1,int,nof_vertices1);
			    }
			}
		      else if( ((polyh1_entity_type == POLYHEDRON_ENTITY)||(polyh1_entity_type == CONVEX_POLYHEDRON)) && 
			       ((polyh2_entity_type == POLYHEDRON_ENTITY)||(polyh2_entity_type == CONVEX_POLYHEDRON)) )
			{
/* 			  if(!p3d_poly_is_convex(polyh1->poly)) */
/* 			    { */
/* 			      PrintInfo(("polyh1 concave !\n"); */
/* 			    } */
/* 			  else if(!p3d_poly_is_convex(polyh2->poly)) */
/* 			    { */
/* 			      PrintInfo(("polyh2 concave !\n"); */
/* 			    } */
/* 			  else */
/* 			    { */
/* 			      PrintInfo(("both polyhs are convex polyhedron\n"); */
/* 			    } */
/*			  v[0]=0.0; v[1]=0.0;v[2]=0.0;*/  
		

/* 			  kcd_get_poly_pos0(polyh2,&polyh2_pos0); */

			  /* Get BwrtW 
			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);*/
			  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);


/*			  kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);
			  kcd_matMultTransfo(mat1,*polyh2_pos0,BwrtA); */
			  /* p3d_matMultXform(mat1,polyh2->pos0,BwrtA); */
			  n_facets = kcd_get_nb_fs(polyh1);
			  /* n_facets = polyh1->poly->nb_faces; */
			  n_facets1 = kcd_get_nb_fs(polyh2);
			  /* n_facets1 = polyh2->poly->nb_faces; */
			  for(u=0;u<n_facets;u++)
			    {
			      nof_vertices1 = kcd_get_nb_pts_in_f(polyh1,u+1);
			      /* nof_vertices1 = p3d_get_nb_points_in_face(polyh1->poly,u+1); */
			      facet_ids1 = MY_ALLOC(int,nof_vertices1);
			      for(w=0;w<nof_vertices1;w++)
				{
				  facet_ids1[w] = kcd_get_i_pt_in_f(polyh1,u+1,w+1);
				  /* facet_ids1[w] = p3d_get_index_point_in_face(polyh1->poly,u+1,w+1); */
				}

			      for(uu=0; uu < n_facets1; uu++)
				{
				  nof_vertices2 = kcd_get_nb_pts_in_f(polyh2,uu+1);
				  /* nof_vertices2 = p3d_get_nb_points_in_face(polyh2->poly,uu+1); */
				  facet_ids2 = MY_ALLOC(int,nof_vertices2);
				  for(w=0;w<nof_vertices2;w++)
				    {
				      facet_ids2[w] = kcd_get_i_pt_in_f(polyh2,uu+1,w+1);
				      /* facet_ids2[w] = p3d_get_index_point_in_face(polyh2->poly,uu+1,w+1); */
				    }

				  kcd_gjk_set_interesting_seed(kcd_gjk_id1,0);

				  /* initialisation of the zpi's */
				  kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2])); 
				  kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2])); 
				  /* end init */

				  collision_exists2 = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
								   kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
								   *AwrtW, *BwrtW, &zpa, &zpb, &min_distance, gjk_tolerance); 
			  
				   if(min_distance < smallest_min_distance )
				     {
				       /* save the necessary information to fill in the report*/
				       closest_obj = k;
				       closest_prim_obj = qit;
				       closest_primitive = pit;
				       kcd_vectCopy(zpa,closest_zpa);
				       kcd_vectCopy(zpb,closest_zpb);
				       smallest_min_distance =  min_distance;
				     }

			/*	  PrintInfo(("POLY <-> POLY  ... \n");
				  PrintInfo(("info zpa = %lf %lf %lf  / zpb =%lf %lf %lf \n", zpa[0], zpa[1],zpa[2],  zpb[0], zpb[1],zpb[2]);  
				  PrintInfo(("info min_distance = %lf \n\n", min_distance);  */

				  /*collision_exists2 =  kcd_gjk_just_intersect(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
					 				      kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
									      BwrtA, &v);*/

				  collision_exists1 = collision_exists1 || collision_exists2;
/* 				  if(collision_exists2) */
/* 				    { */
/* 				      PrintInfo(("baf: facet %i vs facet %i\n",u,uu)); */
/* 				    } */
/* 				  MY_FREE(facet_ids2,int,nof_vertices2); */
				}
			      MY_FREE(facet_ids1,int,nof_vertices1);
			    }
			}
#endif
		      else 
			{
			  PrintInfo(("WARNING : GJK not tested \t")); 
			  collision_exists1 = FALSE;
			  min_distance = P3D_HUGE;		      
			}
		      collision_exists = collision_exists || collision_exists1;

		      /* Report: counts the number of collisions */
		      if(collision_exists) 
			{
			  kcd_add_report(i,k);
 			/*  PrintInfo(("collision between %i and %i\n",i,k)); */
			}		     	          		       
		    } /*visit all primitives of the obstacles */
		}/*visit all obstacles */
	    } /*visit all primitives of the body */ 

	  /* Report mecanism:
	   * Report for each body of each robot the coordinates of the 
	   * closest pair of points	and the corresponding minimum distance
	   *
           * once al the primitives of a body are visited we can fill in the report 
           * 1: fill in the minimum distance
	   * 2: fill in the closest point of the body
	   * 3: fill in the other closest point
	   */
	  /* if nof_polys_r <=0 then we didn't do a collision test so we cannot fill in the report */
	  if( nof_polys_r > 0)
	    {	 
	      /* distance */
	      kcd_set_distance_body_obst(i,j,smallest_min_distance);   
	      
	      /*PrintInfo(("before: a %lf %lf %lf \t b %lf %lf %lf\n", closest_zpa[0],closest_zpa[1],closest_zpa[2],closest_zpb[0],closest_zpb[1],closest_zpb[2]));*/
	      
	      /* point A */
	      polyh1 = XYZ_ENV->robot[i]->o[j]->pol[closest_primitive];
	      kcd_get_prim_rel_pos(polyh1, &AwrtBODY);
	      /*p3d_mat4Print(*AwrtBODY, "AwrtBODY");*/
	      kcd_TransfoPoint(*AwrtBODY,closest_zpa,zpa); 
	      
	      /* point B */
	      polyh2 = XYZ_ENV->o[closest_obj]->pol[closest_prim_obj];
	      kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);
	      /*p3d_mat4Print(*BwrtW, "BwrtW");*/
	      kcd_TransfoPoint(*BwrtW,closest_zpb,zpb);
  	         
	      kcd_set_points_closest_pair(i,j,zpa,zpb);
	     /*PrintInfo(("body %d:  a %lf %lf %lf \t b %lf %lf %lf\t dist = %lf\n",j,zpa[0],zpa[1],zpa[2],zpb[0],zpb[1],zpb[2],smallest_min_distance));*/
	    }
	} /* visit all bodies*/
    } /*visit all robots */



/* The following code is an example of retrieving the report information,
 *
 * This code can also be used for debugging 
 *
 */ 
/*
{ 
 double *distances; 
 kcd_vector3 *r,*s;  
 int nof_bodies,i; 

 nof_bodies = XYZ_ENV->robot[0]->no;
 distances = MY_ALLOC(double,nof_bodies); 
 r = MY_ALLOC(kcd_vector3,nof_bodies); 
 s = MY_ALLOC(kcd_vector3,nof_bodies); 
   
 p3d_col_report_closest_points(XYZ_ENV->robot[0],r,s,distances);  
 for(i=0;i<nof_bodies;i++)  
   PrintInfo(("mo %d:\t  %lf %lf %lf \t so: %lf %lf %lf\tdist =%lf\n",i,r[i][0],r[i][1],r[i][2],s[i][0],s[i][1],s[i][2],distances[i]));  
 PrintInfo(("\n"));  
}  
*/ 


  return collision_exists; 
}


#endif /* GJK_DEBUG */
