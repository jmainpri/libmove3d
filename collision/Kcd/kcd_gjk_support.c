/** **************************************************************************************** **
 ** **************************************************************************************** **
 **     Support functions for GJK                                                            **
 ** **************************************************************************************** **
 ** **************************************************************************************** **/


#include "Util-pkg.h"
#ifdef GJK_DEBUG
#include "P3d-pkg.h"
#endif
#include "Collision-pkg.h"


/* typedef double kcd_vector3[3]; */
int *interesting_seed = NULL;


void kcd_gjk_support_initialize_interesting_seed(int size_to_allocate)
{
  int it;

  interesting_seed = MY_ALLOC(int,size_to_allocate);
  for(it=0;it<size_to_allocate;it++)
    interesting_seed[it] = 0;
}

void kcd_gjk_support_resize_interesting_seed(int orig_size,int size_to_add)
{
  int it;

  interesting_seed = MY_REALLOC(interesting_seed,int,
				orig_size,orig_size+size_to_add);
  for(it=orig_size;it<orig_size+size_to_add;it++)
    interesting_seed[it] = 0;
}

void kcd_gjk_set_interesting_seed(int this_one, int to_this)
{
  if(interesting_seed)
    interesting_seed[this_one] = to_this;
  else
    PrintInfo(("WARNING (kcd_gjk_set_interesting_seed): interesting_seed is NULL \n"));
}

void kcd_gjk_support_cleanup_interesting_seed(int size_to_destroy)
{
  MY_FREE(interesting_seed,int,size_to_destroy);
  interesting_seed = NULL;
}

/* ************************************************************************* *
 * Function:  kcd_gjk_support()
 * ARGS IN :  void pointer in case of a primitive   
 *            list of facet vertices in case of a convex facet 
 *            list of triangle vertices  in case of a concave facet                
 *            is_prim: true if object is a primitive,                              
 *                     false in case of triangle or facet                          
 *            Vector gjk_in_vector, support vector related to GJK algorithm        
 * ARGS OUT : Point support_point, the support point needed in the 
 *            GJK algorithm   
 * PRECOND  : 0 < facet_ids[k] for each k=0..nof_vertices                         
 * ************************************************************************* */
void kcd_gjk_support(int kcd_gjk_obj_id, int nof_vertices, int *facet_ids, void *poly, kcd_vector3 gjk_in_vector, kcd_vector3 *support_point)
{  
  double ndotprod,dotprod,sphere_vector_length,cone_vector_length,cone_vector_param,cylinder_vector_param,gjk_scale,cone_rhs;
  int vert_it,next_it;
  kcd_vector3 *the_points=NULL;
  /*  int gjk_select=1,np;*/
  int the_entity_type = kcd_get_poly_entity_type(poly);
  /*  double dotprod1, dotprod2;*/
  double kcd_ext_data, kcd_ext_height,x_length,y_length,z_length;



/* NOTE: GJK is not yet compatible with KCD small volumes !!*/

#ifdef GJK_DEBUG
  if(FALSE){}
#else

  if(get_entity_type(kcd_gjk_obj_id) == KCD_SMALL_VOLUME_BOX)
    {
      if(LNEQ(gjk_in_vector[0],0.0))
	(*support_point)[0] = -get_kcd_bb_half_size(kcd_gjk_obj_id,0); /* returns bb->d[0]; */
      else
	(*support_point)[0] =  get_kcd_bb_half_size(kcd_gjk_obj_id,0); /* returns bb->d[0]; */
      if(LNEQ(gjk_in_vector[1],0.0))
	(*support_point)[1] = -get_kcd_bb_half_size(kcd_gjk_obj_id,1); /* returns bb->d[1]; */
      else
	(*support_point)[1] =  get_kcd_bb_half_size(kcd_gjk_obj_id,1); /* returns bb->d[1]; */
      if(LNEQ(gjk_in_vector[2],0.0)) 
	(*support_point)[2] = -get_kcd_bb_half_size(kcd_gjk_obj_id,2); /* returns bb->d[2]; */
      else
	(*support_point)[2] =  get_kcd_bb_half_size(kcd_gjk_obj_id,2); /* returns bb->d[2]; */
    }

#endif /*GJK_DEBUG */

#ifdef CP_IS_SOLID
  else if( (the_entity_type == SPHERE_ENTITY)||(the_entity_type == CYLINDER_ENTITY)||(the_entity_type == CUBE_ENTITY)||(the_entity_type == BOX_ENTITY)||(the_entity_type == CONE_ENTITY)||(the_entity_type == CONVEX_POLYHEDRON) ||(the_entity_type == KCD_OBB)   )
#else
  else if( (the_entity_type == SPHERE_ENTITY)||(the_entity_type == CYLINDER_ENTITY)||(the_entity_type == CUBE_ENTITY)||(the_entity_type == BOX_ENTITY)||(the_entity_type == CONE_ENTITY) ||(the_entity_type == KCD_OBB)   )
#endif
    {
      /* we have a primitive as input */
      switch(the_entity_type) {
      case SPHERE_ENTITY:
	{
	  sphere_vector_length = kcd_vectNorm(gjk_in_vector);
	  if( GNEQ(sphere_vector_length,0.0) )
	    {
	      kcd_get_solid_r1(poly,&kcd_ext_data);
	      gjk_scale = kcd_ext_data / sphere_vector_length;
	      (*support_point)[0] = gjk_in_vector[0] * gjk_scale;
	      (*support_point)[1] = gjk_in_vector[1] * gjk_scale;
	      (*support_point)[2] = gjk_in_vector[2] * gjk_scale;
	    }
	  else
	    {
	      (*support_point)[0] = 0.0;
	      (*support_point)[1] = 0.0;
	      (*support_point)[2] = 0.0;
	    }
	  break;
	}
      case CONE_ENTITY:
	{
	  cone_vector_param = sqrt( gjk_in_vector[0] * gjk_in_vector[0] + gjk_in_vector[1] * gjk_in_vector[1] );
	  cone_vector_length = kcd_vectNorm(gjk_in_vector);
	  kcd_get_solid_s(poly,&kcd_ext_data);
	  kcd_get_solid_h(poly,&kcd_ext_height);
	  cone_rhs = cone_vector_length * kcd_ext_data;

/* 	  if(LEQ(gjk_in_vector[2],0.0)) */
/* 	    { */
/* 	      if( LEQ((-gjk_in_vector[2]),cone_rhs) ) */
/* 		{ */
/* 		  if(EQ(cone_vector_param,0.0)) */
/* 		    { */
/* 		      (*support_point)[0] =   0.0; */
/* 		      (*support_point)[1] =   0.0; */
/* 		    } */
/* 		  else */
/* 		    { */
/* 		      kcd_get_solid_r2(poly,&kcd_ext_data); */
/* 		      gjk_scale = kcd_ext_data / ( cone_vector_param  ); */
/* 		      (*support_point)[0] =   gjk_in_vector[0] * gjk_scale; */
/* 		      (*support_point)[1] =   gjk_in_vector[1] * gjk_scale; */
/* 		    } */
/* 		  (*support_point)[2] = - kcd_ext_height/2.0; */
/* 		} */
/* 	      else */
/* 		{ */
/* 		  if(EQ(cone_vector_param,0.0)) */
/* 		    { */
/* 		      (*support_point)[0] =   0.0; */
/* 		      (*support_point)[1] =   0.0; */
/* 		    } */
/* 		  else */
/* 		    { */
/* 		      kcd_get_solid_r1(poly,&kcd_ext_data); */
/* 		      gjk_scale = kcd_ext_data / ( cone_vector_param ); */
/* 		      (*support_point)[0] =   gjk_in_vector[0] * gjk_scale; */
/* 		      (*support_point)[1] =   gjk_in_vector[1] * gjk_scale; */
/* 		    } */
/* 		  (*support_point)[2] =   kcd_ext_height/2.0; */
/* 		} */
/* 	    } */
/* 	  else */
/* 	    { */

	      if( LEQ((gjk_in_vector[2]),cone_rhs) )
		{
		  /* bottom */
/* 		  PrintInfo(("cone case 2.1, gjk_in_vector[2] = %f\n",gjk_in_vector[2])); */
		  if(EQ(cone_vector_param,0.0))
		    {
		      /* at the center */
		      /* PrintInfo(("center\n")); */
		      (*support_point)[0] =   0.0;
		      (*support_point)[1] =   0.0;
		    }
		  else
		    {
		      /* at border */
		      /* PrintInfo(("border\n")); */
		      kcd_get_solid_r2(poly,&kcd_ext_data);
		      gjk_scale = kcd_ext_data / ( cone_vector_param );
		      /* gjk_scale = the_prim->other_radius / ( cone_vector_param * 2.0 ); */
		      (*support_point)[0] =   gjk_in_vector[0] * gjk_scale;
		      (*support_point)[1] =   gjk_in_vector[1] * gjk_scale;
		    }
		  (*support_point)[2] = - kcd_ext_height/2.0;
		}
	      else
		{
		  /* top */
/* 		  PrintInfo(("cone case 2.2, gjk_in_vector[2] = %f\n",gjk_in_vector[2])); */
		  if(EQ(cone_vector_param,0.0))
		    {
		      /* at the center */
		      /* PrintInfo(("center\n")); */
		      (*support_point)[0] =   0.0;
		      (*support_point)[1] =   0.0;
		    }
		  else
		    {
		      /* at border */
		      /* PrintInfo(("border\n")); */
		      kcd_get_solid_r1(poly,&kcd_ext_data);
		      gjk_scale = kcd_ext_data / ( cone_vector_param );
		      /* gjk_scale = the_prim->radius / ( cone_vector_param * 2.0 ); */
		      (*support_point)[0] =   gjk_in_vector[0] * gjk_scale;
		      (*support_point)[1] =   gjk_in_vector[1] * gjk_scale;
		    }
		  (*support_point)[2] =   kcd_ext_height/2.0;
		}
/* 	    } */
	  break;
	}
      case CYLINDER_ENTITY:
	{
	  /* cylinder lies with height along Z-axis */
	  kcd_get_solid_h(poly,&kcd_ext_height);
	  cylinder_vector_param = sqrt( gjk_in_vector[0] * gjk_in_vector[0] + gjk_in_vector[1] * gjk_in_vector[1] );
	  if( GNEQ(cylinder_vector_param,0.0) )
	    {
	      kcd_get_solid_r1(poly,&kcd_ext_data);
	      gjk_scale = kcd_ext_data / cylinder_vector_param;
	      (*support_point)[0] = gjk_in_vector[0] * gjk_scale;
	      (*support_point)[1] = gjk_in_vector[1] * gjk_scale;
	      if(LNEQ(gjk_in_vector[2],0.0)) 
		{
		  (*support_point)[2] = -kcd_ext_height/2.0;

		}
	      else
		{
		  (*support_point)[2] =  kcd_ext_height/2.0;
		}
	    }
	  else
	    {
	      (*support_point)[0] = 0.0;
	      (*support_point)[1] = 0.0;
	      if(LNEQ(gjk_in_vector[2],0.0)) 
		{
		  (*support_point)[2] = -kcd_ext_height/2.0;
		}
	      else
		{
		  (*support_point)[2] =  kcd_ext_height/2.0;
		}
	    }
	  break;
	}
      case CUBE_ENTITY:
	{
	  kcd_get_solid_x(poly,&x_length);
	  if(LNEQ(gjk_in_vector[0],0.0))
	    (*support_point)[0] = -x_length/2.0;
	  else
	    (*support_point)[0] =  x_length/2.0;
	  if(LNEQ(gjk_in_vector[1],0.0))
	    (*support_point)[1] = -x_length/2.0;
	  else
	    (*support_point)[1] =  x_length/2.0;
	  if(LNEQ(gjk_in_vector[2],0.0))
	    (*support_point)[2] = -x_length/2.0;
	  else
	    (*support_point)[2] =  x_length/2.0;
	  break;
	}
      case BOX_ENTITY:
	{
	  kcd_get_solid_xyz(poly,&x_length,&y_length,&z_length);
	  if(LNEQ(gjk_in_vector[0],0.0))
	    (*support_point)[0] = -x_length/2.0;
	  else
	    (*support_point)[0] =  x_length/2.0;
	  if(LNEQ(gjk_in_vector[1],0.0))
	    (*support_point)[1] = -y_length/2.0;
	  else
	    (*support_point)[1] =  y_length/2.0;
	  if(LNEQ(gjk_in_vector[2],0.0)) 
	    (*support_point)[2] = -z_length/2.0;
	  else
	    (*support_point)[2] =  z_length/2.0;
	  break;
	}
#ifdef CP_IS_SOLID
      case CONVEX_POLYHEDRON:
	{
	  kcd_get_pt_arr(poly,&the_points);
	  np = kcd_get_nb_pts(poly);

	  dotprod1 = kcd_vectDotProd(the_points[0],gjk_in_vector);
	  /* visit all vertices */
	  for (vert_it = 1; vert_it < np ; vert_it++) 
	    {
	      dotprod2 = kcd_vectDotProd(the_points[vert_it],gjk_in_vector);
	      if(dotprod2 > dotprod1) 
		{ 
		  gjk_select = vert_it; 
		  dotprod1 = dotprod2; 
		}
	    }
	  (*support_point)[0] = the_points[gjk_select][0];
	  (*support_point)[1] = the_points[gjk_select][1];
	  (*support_point)[2] = the_points[gjk_select][2];
	  break;
	}
#else
	/* treat convex polyhedron as a collection of convex facets, below */
/*       case CONCAVE_POLYHEDRON: */
/* 	{ */
/* 	  PrintInfo(("error KCD (kcd_gjk_support): CONCAVE_POLYHEDRON should be collection of triangles and convex facets \n")); */
/* 	  break; */
/* 	} */
#endif
      case POLYHEDRON_ENTITY:
	{
	  PrintInfo(("error KCD (kcd_gjk_support): polyhedron entity type should be CONVEX or CONCAVE \n"));
	  break;
	}
      case KCD_OBB:
	{
	  /* kcd_get_solid_xyz(poly,&x_length,&y_length,&z_length);*/
	  if(LNEQ(gjk_in_vector[0],0.0))
	    (*support_point)[0] = -all_bbs[kcd_gjk_obj_id]->d[0];
	  else
	    (*support_point)[0] = all_bbs[kcd_gjk_obj_id]->d[0];
	  if(LNEQ(gjk_in_vector[1],0.0))
	    (*support_point)[1] = -all_bbs[kcd_gjk_obj_id]->d[1];
	  else
	    (*support_point)[1] = all_bbs[kcd_gjk_obj_id]->d[1];
	  if(LNEQ(gjk_in_vector[2],0.0)) 
	    (*support_point)[2] = -all_bbs[kcd_gjk_obj_id]->d[2];
	  else
	    (*support_point)[2] = all_bbs[kcd_gjk_obj_id]->d[2]; 
	  break; 	  
	}
      default:
	break;
      }
    }
  else
    {
      /* we have a convex facet or a triangle as input */
      vert_it = interesting_seed[kcd_gjk_obj_id];
#ifdef GJK_DEBUG
      if(p3d_col_get_mode()==p3d_col_mode_gjk)
	/* debug only: */ vert_it = 0;
#endif
      kcd_get_pt_arr(poly,&the_points);
      /* visit all facet_vertices[vert_it] */
      dotprod = kcd_vectDotProd(the_points[facet_ids[vert_it]-1],gjk_in_vector);
      if(vert_it+1 < nof_vertices)
	next_it = vert_it+1;
      else
	next_it = 0;
      ndotprod = kcd_vectDotProd(the_points[facet_ids[next_it]-1],gjk_in_vector);
      if(ndotprod > dotprod)
	{
	  do
	    {//PrintInfo(("NIC dot %f ndot %f\n",dotprod,ndotprod));
	      dotprod = ndotprod;
	      vert_it = next_it;
	      if(++next_it == nof_vertices)
		next_it = 0;
	      ndotprod = kcd_vectDotProd(the_points[facet_ids[next_it]-1],gjk_in_vector);
	    }while(ndotprod > (dotprod+FUZZ));
	}
      else
	{
	  if(vert_it == 0)
	    next_it = nof_vertices-1;
	  else
	    next_it = vert_it-1 ;
	  ndotprod = kcd_vectDotProd(the_points[facet_ids[next_it]-1],gjk_in_vector);
	  while(ndotprod > (dotprod+FUZZ))
	    {//PrintInfo(("NIC  dot %f ndot %f\n",dotprod,ndotprod));
	      dotprod = ndotprod;
	      vert_it = next_it;
	      if(next_it == 0) 
		next_it = nof_vertices-1; 
	      else
		next_it-- ;
	      ndotprod = kcd_vectDotProd(the_points[facet_ids[next_it]-1],gjk_in_vector);
	    }
	}
      interesting_seed[kcd_gjk_obj_id]= vert_it; /* for efficiency reasons */
      /* return point */
      (*support_point)[0] = the_points[facet_ids[vert_it]-1][0];
      (*support_point)[1] = the_points[facet_ids[vert_it]-1][1];
      (*support_point)[2] = the_points[facet_ids[vert_it]-1][2];
    }
}
