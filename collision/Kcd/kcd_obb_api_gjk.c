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


/* Modification Pepijn Raeymaekers july 2001
 * this function has been rewritten with a switch( with_report)
 */

/*! 
  \file kcd_obb_api_gjk.c
  \brief interface between OBB's and GJK 

  For two given OBB boxes GJK is called and depending on the with_report paramameter
  another report is created
  \sa kcd_gjk_just_intersect(), kcd_gjk_closest_pair()
*/
int kcd_gjk_intersect(kcd_bb *obbox1, kcd_bb *obbox2, kcd_vector3 *v, int with_report, double *min_distance, double gjk_tolerance)
{
  int entity_type1 = get_entity_type(obbox1->bb_id_in_all_bbs);
  int entity_type2 = get_entity_type(obbox2->bb_id_in_all_bbs);
  int in_collision = FALSE;  
  kcd_vector3 zpa,zpb,tzpb, tzpa;
  kcd_matrix4 *AwrtW,*BwrtW;
  kcd_matrix4 BwrtA,rmat,*polyh1_poly_pos,*polyh2_poly_pos,*polyh1_pos0,*polyh2_pos0;
  int nof_vertices1=0;
  int *facet_ids1=NULL;
  int kcd_gjk_id1;
  void *polyh1=NULL;
  int nof_vertices2=0;
  int *facet_ids2=NULL;
  int kcd_gjk_id2;
  void *polyh2=NULL;
  double current_exact_dist = P3D_HUGE;

  polyh1 = obbox1->pol;
  polyh2 = obbox2->pol;

  switch (with_report)
    {
    case  DISTANCE_ESTIMATE:
      {  
	*min_distance = 0.0;
	/* don't check 2 boxes, since we checked the 2 OBBs already */
	if( ((entity_type1 == CUBE_ENTITY)||(entity_type1 == BOX_ENTITY)||(entity_type1 == KCD_SMALL_VOLUME_BOX)) &&
	    ((entity_type2 == CUBE_ENTITY)||(entity_type2 == BOX_ENTITY)||(entity_type2 == KCD_SMALL_VOLUME_BOX)) )
	  {    
	    
	    in_collision = TRUE;  
	  }	
	else {
	  if((entity_type1 == KCD_SMALL_VOLUME_BOX) || (entity_type2 == KCD_SMALL_VOLUME_BOX))
	    {
	      in_collision = TRUE;
	    }
	  else {
	    if((obbox1->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox1->what_is_inside == WAS_POLYHEDRON))
	      {
		nof_vertices1 = obbox1->nof_facet_vertices;
		facet_ids1 = obbox1->facet_vertices;
		if(obbox1->is_robot_prim)
		  {			  
		    kcd_get_prim_abs_pos(polyh1,TRUE,&AwrtW);
		    /* take first vertex of polyhedron */
		    kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2]));	
		  }
		else
		  {			  
		    kcd_get_prim_abs_pos(polyh1,FALSE,&AwrtW);
		    /* take first vertex of polyhedron */
		    kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2]));	
		  }
	      }
	    else if(obbox1->entity_type != KCD_SMALL_VOLUME_BOX)
	      {
		if(obbox1->is_robot_prim)
		  {	
		    kcd_get_prim_abs_pos(polyh1,TRUE,&AwrtW);
		    /* origin is inside solid primitive */
		    zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;	
		  }
		else
		  {		
		    kcd_get_prim_abs_pos(polyh1,FALSE,&AwrtW);
		    /* origin is inside solid primitive */
		    zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;			  			
		  }
	      }
	    kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;
	    if((obbox2->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox2->what_is_inside == WAS_POLYHEDRON))
	      {
		nof_vertices2 = obbox2->nof_facet_vertices;
		facet_ids2 = obbox2->facet_vertices;
		if(obbox2->is_robot_prim)
		  {		
		    kcd_get_prim_abs_pos(polyh2,TRUE,&BwrtW);
		    /* take first vertex of polyhedron */
		    kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2]));	
		  }
		else
		  {       
		    kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);
		    /* take first vertex of polyhedron */
		    kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2]));	
		  }
	      }
	    else if(obbox2->entity_type != KCD_SMALL_VOLUME_BOX)
	      {
		if(obbox2->is_robot_prim)
		  {	
		    kcd_get_prim_abs_pos(polyh2,TRUE,&BwrtW);		 
		    /* origin is inside solid primitive */
		    zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;       
		  }
		else
		  {       
		    kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);
		    /* origin is inside solid primitive */
		    zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;		
		  }
	      }
	    kcd_gjk_id2 = obbox2->bb_id_in_all_bbs; 
	    in_collision = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
						kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
						*AwrtW, *BwrtW, &zpa, &zpb, min_distance, gjk_tolerance); 
	  }
	}
	break;
      }
    case  DISTANCE_EXACT:
      {
	if((entity_type1 == KCD_SMALL_VOLUME_BOX) || (entity_type2 == KCD_SMALL_VOLUME_BOX))
	  {
	    in_collision = TRUE;
	  }
	else {
	  current_exact_dist = *min_distance;     
	  //	PrintInfo((" min %d , cur %d\n", *min_distance, current_exact_dist));
	  *min_distance = 0.0;
	  if((obbox1->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox1->what_is_inside == WAS_POLYHEDRON))
	    {
	      nof_vertices1 = obbox1->nof_facet_vertices;
	      facet_ids1 = obbox1->facet_vertices;
	      if(obbox1->is_robot_prim)
		{			  
		  kcd_get_prim_abs_pos(polyh1,TRUE,&AwrtW);
		  /* take first vertex of polyhedron */
		  kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2]));       
		}
	      else
		{			  
		  kcd_get_prim_abs_pos(polyh1,FALSE,&AwrtW);
		  /* take first vertex of polyhedron */
		  kcd_get_pt(polyh1,facet_ids1[0],&(zpa[0]),&(zpa[1]),&(zpa[2]));	
		}
	    }
	  else if(obbox1->entity_type != KCD_SMALL_VOLUME_BOX)
	    {
	      if(obbox1->is_robot_prim)
		{	
		  kcd_get_prim_abs_pos(polyh1,TRUE,&AwrtW);
		  /* origin is inside solid primitive */
		  zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;	
		}
	      else
		{		
		  kcd_get_prim_abs_pos(polyh1,FALSE,&AwrtW);
		  /* origin is inside solid primitive */
		  zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;			  			
		}
	    }
	  kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;
	  if((obbox2->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox2->what_is_inside == WAS_POLYHEDRON))
	    {
	      nof_vertices2 = obbox2->nof_facet_vertices;
	      facet_ids2 = obbox2->facet_vertices;
	      if(obbox2->is_robot_prim)
		{		
		  kcd_get_prim_abs_pos(polyh2,TRUE,&BwrtW);
		  /* take first vertex of polyhedron */
		  kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2]));	
		}
	      else
		{       
		  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);
		  /* take first vertex of polyhedron */
		  kcd_get_pt(polyh2,facet_ids2[0],&(zpb[0]),&(zpb[1]),&(zpb[2]));	
		}
	    }
	  else if(obbox2->entity_type != KCD_SMALL_VOLUME_BOX)
	    {
	      if(obbox2->is_robot_prim)
		{	
		  kcd_get_prim_abs_pos(polyh2,TRUE,&BwrtW);		 
		  /* origin is inside solid primitive */
		  zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;       
		}
	      else
		{       
		  kcd_get_prim_abs_pos(polyh2,FALSE,&BwrtW);
		  /* origin is inside solid primitive */
		  zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;		
		}
	    }
	  kcd_gjk_id2 = obbox2->bb_id_in_all_bbs; 
	  in_collision = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
					      kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
					      *AwrtW, *BwrtW, &zpa, &zpb, min_distance, gjk_tolerance); 
//	  PrintInfo(("\tmin_distance %.0lf\n", *min_distance));
	  /* fill in the report */
//  	  kcd_dist_set_min_dist(&current_exact_dist);
	  if(*min_distance <  kcd_dist_get_min_dist())
//	  if(*min_distance < current_exact_dist)
	    { 
	      kcd_dist_set_min_dist(*min_distance); 
	      kcd_TransfoPoint(*BwrtW, zpb, tzpb);  
	      kcd_TransfoPoint(*AwrtW, zpa, tzpa);
	      //  PrintInfo(("we fill in pa %.0lf %.0lf %.0lf \t pb %.0lf %.0lf %.0lf\n",zpa[0],zpa[1],zpa[2],zpb[0],zpb[1], zpb[2] ));  
	      //  PrintInfo(("\twe fill in tpa %.0lf %.0lf %.0lf tpb %.0lf %.0lf %.0lf\n",tzpa[0],tzpa[1],tzpa[2],tzpb[0],tzpb[1], tzpb[2]));
	      //  PrintInfo((" we set%.0lf",*min_distance));
	      kcd_dist_set_closest_points(tzpa,tzpb);
	    }
	}
	break;
      }

    default:
      {

	/* don't check 2 boxes, since we checked the 2 OBBs already */
	if( ((entity_type1 == CUBE_ENTITY)||(entity_type1 == BOX_ENTITY)||(entity_type1 == KCD_SMALL_VOLUME_BOX)) &&
	    ((entity_type2 == CUBE_ENTITY)||(entity_type2 == BOX_ENTITY)||(entity_type2 == KCD_SMALL_VOLUME_BOX)) )
	  {          
	    in_collision = TRUE;  
	  }
	else {
	  if((entity_type1 == KCD_SMALL_VOLUME_BOX) || (entity_type2 == KCD_SMALL_VOLUME_BOX))
	    {
	      in_collision = TRUE;
	    }
	  else {
	    if((obbox1->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox1->what_is_inside == WAS_POLYHEDRON))
	      {
		nof_vertices1 = obbox1->nof_facet_vertices;
		facet_ids1 = obbox1->facet_vertices;
		if(obbox1->is_robot_prim)
		  {	 
		    kcd_get_prim_abs_pos(polyh1,TRUE,&polyh1_poly_pos);
		    kcd_matInvertTransfo(*polyh1_poly_pos,rmat);	  
		  }
		else
		  {	
		    kcd_get_prim_abs_pos(polyh1,FALSE,&polyh1_pos0);
		    kcd_matInvertTransfo(*polyh1_pos0,rmat);	   
		  }
	      }
	    else if(obbox1->entity_type != KCD_SMALL_VOLUME_BOX)
	      {
		if(obbox1->is_robot_prim)
		  {	 
		    kcd_get_prim_abs_pos(polyh1,TRUE,&polyh1_poly_pos);
		    kcd_matInvertTransfo(*polyh1_poly_pos,rmat);	   
		  }
		else
		  {	  
		    kcd_get_prim_abs_pos(polyh1,FALSE,&polyh1_pos0);
		    kcd_matInvertTransfo(*polyh1_pos0,rmat);	   
		  }
	      }
	    kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;
	    
	    if((obbox2->entity_type != KCD_SMALL_VOLUME_BOX) && (obbox2->what_is_inside == WAS_POLYHEDRON))
	      {
		nof_vertices2 = obbox2->nof_facet_vertices;
		facet_ids2 = obbox2->facet_vertices;
		if(obbox2->is_robot_prim)
		  {	
		    kcd_get_prim_abs_pos(polyh2,TRUE,&polyh2_poly_pos);
		    kcd_matMultTransfo(rmat,*polyh2_poly_pos,BwrtA);	   
		  }
		else
		  {		  
		    kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);
		    kcd_matMultTransfo(rmat,*polyh2_pos0,BwrtA);	   
		  }
	      }
	    else if(obbox2->entity_type != KCD_SMALL_VOLUME_BOX)
	      {
		if(obbox2->is_robot_prim)
		  {	   
		    kcd_get_prim_abs_pos(polyh2,TRUE,&polyh2_poly_pos);
		    kcd_matMultTransfo(rmat,*polyh2_poly_pos,BwrtA);	   
		  }
		else
		  {	 	  
		    kcd_get_prim_abs_pos(polyh2,FALSE,&polyh2_pos0);
		    kcd_matMultTransfo(rmat,*polyh2_pos0,BwrtA);	   
		  }
	      }
	    kcd_gjk_id2 = obbox2->bb_id_in_all_bbs; 
	    in_collision = kcd_gjk_just_intersect(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
						  kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
						  BwrtA, v);   
	  }
	}
      }
    }
  /* B Kineo Carl 22.02.2002 */
  /* if in collision, write report on collision pair */
  if(in_collision)
    {
      kcd_set_pairInCollision(polyh1, obbox1->ext_pol_id, polyh2, obbox2->ext_pol_id);
    }
  /* E Kineo Carl 22.02.2002 */

  /* return result */
  return in_collision;
}
