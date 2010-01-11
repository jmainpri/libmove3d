#include "Util-pkg.h"
//#include "P3d-pkg.h"
#include "Collision-pkg.h"

typedef double *double_p;

static void split_and_recurse_for_obbs(void *m3d_poly_it,kcd_vector3 *all_obbs_as_one_polyh_the_points,kcd_bb_p *orig_obbs, int *nof_the_facet_vertices, kcd_index_p *the_facet_vertices, int parent_rank_number, kcd_bb_p parent_p, kcd_matrix3 *cij_by_facet, kcd_vector3 *evects,kcd_vector3 momentH,int face_num,int *recursed_facets,kcd_vector3 *facet_moments,double *facet_area,int is_robot, int is_convex)
{
  /*kcd_index *these_vertices = NULL;*/
/*  int nof_vertices = 0;*/
  int is_leaf = FALSE;
  kcd_bb_p last_bbs_pile1=NULL,last_bbs_pile2=NULL;
  double axdmp,total_facet_area1 = 0.0, total_facet_area2 = 0.0;
  kcd_vector3 Teller1,Teller2;
  int n1,n2,facet_nr,i,ii,jj;
  int *faces1=NULL;
  int *faces2=NULL;
  kcd_vector3 momentH1;
  kcd_vector3 momentH2;
  kcd_matrix3 cov_matrix;
  kcd_vector3 center1;
/*  kcd_vector3 placed_center1;*/
  kcd_vector3 evects1[3];
  kcd_vector3 trans_evects1[3];
/*  kcd_vector3 orient1[3];*/
  kcd_vector3 center2;
  /*kcd_vector3 placed_center2;*/
  kcd_vector3 evects2[3];
  kcd_vector3 trans_evects2[3];  
/*  kcd_vector3 orient2[3];*/
  kcd_matrix3 cij1,cij2;
  double box_x,box_y,box_z;
  int child1_rank_number = 0,child2_rank_number = 0; //set to zero to avoid warnings
#ifdef OBB_EVALUATION
  FILE *OBBSPLIT;
  int we_can_write = TRUE;
#endif
  /* B Kineo Carl 25.02.2002 */
  int ext_p_id = -1;
  /* E Kineo Carl 25.02.2002 */
  /* B Kineo Carl 27.02.2002 */
  int ext_obj_id = -1;
  /* E Kineo Carl 27.02.2002 */

#ifdef OBB_EVALUATION
  if(!(OBBSPLIT = fopen("obbsplit.cvg","a"))){
    PrintError(("MP: kcd_enclose_obb.c (split_and_recurse): can't open file obbsplit.cvg\n"));
    we_can_write = FALSE;
  }
#endif

  init_vector_zero(Teller1);
  init_vector_zero(Teller2);
  /* project approximate mean point onto splitting axis, and get coord. */
  axdmp = (evects[0][0] * momentH[0] + evects[1][0] * momentH[1] + evects[2][0] * momentH[2]);

  n1 = 0; 
  n2 = 0;
  for(i=0;i<face_num;i++)
    {
      facet_nr = recursed_facets[i];
      
      if (((evects[0][0]*facet_moments[facet_nr][0] 
	    + evects[1][0]*facet_moments[facet_nr][1] 
	    + evects[2][0]*facet_moments[facet_nr][2])/facet_area[facet_nr] < axdmp)
	  && ((face_num!=2) || ((face_num==2) && (facet_nr==0))))    
	{
	  /* accumulate first and second order moments for group 1 */
	  faces1 = MY_REALLOC(faces1,int,n1,n1+1);
	  n1++;
	  faces1[n1-1]=facet_nr;
	  kcd_vectAdd(Teller1,facet_moments[facet_nr],Teller1);
	  total_facet_area1 += facet_area[facet_nr];
	}
     else
       {
	 /* accumulate first and second order moments for group 2 */
	 faces2 = MY_REALLOC(faces2,int,n2,n2+1);
	 n2++;
	 faces2[n2-1]=facet_nr;
	 kcd_vectAdd(Teller2,facet_moments[facet_nr],Teller2);
	 total_facet_area2 += facet_area[facet_nr];
       }
    }
  /*   n2 = face_num - n1; */

  /* error check! */
  if (((n1 == 0)&&(face_num > 1)) || ((n1 == face_num)&&(face_num > 1)))
    {
      if(n1 != 0) MY_FREE(faces1,int,n1); faces1 = NULL;
      if(n2 != 0) MY_FREE(faces2,int,n2); faces2 = NULL;
#ifdef OBB_EVALUATION
      if(we_can_write) fprintf(OBBSPLIT,"partitioning failed: n1=%i, n2=%i\n",n1,n2);
#endif      
      /* our partitioning has failed: all the triangles fell into just */
      /* one of the groups.  So, we arbitrarily partition them into */
      /* equal parts, and proceed. */
      init_vector_zero(Teller1);
      init_vector_zero(Teller2);
      total_facet_area1 = 0.0;
      total_facet_area2 = 0.0;
      n1 = face_num/2;
      faces1 = MY_ALLOC(int,n1);
      for(i=0;i<n1;i++)
	{
	  facet_nr = recursed_facets[i];
	  faces1[i]=facet_nr;
	  kcd_vectAdd(Teller1,facet_moments[facet_nr],Teller1);
	  total_facet_area1 += facet_area[facet_nr];
	}
      n2 = face_num - n1;
      faces2 = MY_ALLOC(int,n2);
      for(i=n1;i<face_num;i++)
	{
	  facet_nr = recursed_facets[i];
	  faces2[i-n1]=facet_nr;
	  kcd_vectAdd(Teller2,facet_moments[facet_nr],Teller2);
	  total_facet_area2 += facet_area[facet_nr];
	}
    }
#ifdef OBB_EVALUATION
  else
    {
      if(we_can_write) fprintf(OBBSPLIT,"partitioning succeeded: n1=%i, n2=%i\n",n1,n2);
    }
#endif
  if(!EQ(total_facet_area1,0.0))
    kcd_vectScale(Teller1,momentH1,1.0/total_facet_area1);
  else
    {
      kcd_vectCopy(Teller1,momentH1);
    }
  if(!EQ(total_facet_area2,0.0))
    kcd_vectScale(Teller2,momentH2,1.0/total_facet_area2);
  else
    {
      kcd_vectCopy(Teller2,momentH2);
    }
  if(0<n1)
    kcd_mat3Copy(cij_by_facet[faces1[0]],cij1);
  for(facet_nr=1;facet_nr<n1;facet_nr++)
    {
      kcd_mat3Add(cij1,cij_by_facet[faces1[facet_nr]],cij1);
    }
  if(0<n2)
    kcd_mat3Copy(cij_by_facet[faces2[0]],cij2);
  for(facet_nr=1;facet_nr<n2;facet_nr++)
    {
      kcd_mat3Add(cij2,cij_by_facet[faces2[facet_nr]],cij2);
    }

  /* child number 1 */
  if(n1>0)
    {
      /* is_leaf = (n1 > 1); */
      kcd_get_covariance(Teller1,total_facet_area1,cij1,cov_matrix);
      kcd_get_eigenvalues(evects1,cov_matrix); 

      /* since this is not a robot and since the vertices are on their placement */
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  trans_evects1[ii][jj] = evects1[jj][ii];
      /* put obb away */
      kcd_get_obb_shape_facets(&box_x, &box_y, &box_z, &center1, all_obbs_as_one_polyh_the_points,nof_the_facet_vertices,the_facet_vertices, n1, faces1, trans_evects1); /* 16 11 */
      if(n1==1)
	{
	  /* don't make a new OBB, just make a connection with the existing one (== top of an original OBB(-Tree)) */
	  make_obb_tree_connection(orig_obbs[faces1[0]],parent_p);
	  /* put obb data in the aabb-tree-leaf data structure */
 	}
      else
	{
	  child1_rank_number = put_obb_away(FALSE,box_x, box_y, box_z, center1, trans_evects1, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL, OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	  make_tree_connection(child1_rank_number,parent_p);
	}
      /* end modif. */
      last_bbs_pile1 = last_bbs_pile;
	
    }

  /* child number 2 */
  if(n2>0)
    {
      /* is_leaf = (n2 > 1); */
      kcd_get_covariance(Teller2,total_facet_area2,cij2,cov_matrix);
      kcd_get_eigenvalues(evects2,cov_matrix); /* evects == box.pR */
      /* since this is not a robot and since the vertices are on their placement */
      /* put orientation right w.r.t. work frame */
      
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  trans_evects2[ii][jj] = evects2[jj][ii];
      kcd_get_obb_shape_facets(&box_x, &box_y, &box_z, &center2, all_obbs_as_one_polyh_the_points,nof_the_facet_vertices,the_facet_vertices,n2,faces2,trans_evects2); 

      if(n2==1)
	{
	  /* don't make a new OBB, just make a connection with the existing one (== top of an original OBB(-Tree)) */
	  make_obb_tree_connection(orig_obbs[faces2[0]],parent_p);
 	}
      else
	{
	  child2_rank_number = put_obb_away(FALSE,box_x, box_y, box_z, center2, trans_evects2, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL, OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	  make_tree_connection(child2_rank_number,parent_p);
	}
      /* end modif. */
      last_bbs_pile2 = last_bbs_pile;
	
    }
#ifdef OBB_EVALUATION
  if(we_can_write) fclose(OBBSPLIT);
#endif
  /* continue recursively */
  if(n1>1)
    split_and_recurse_for_obbs(m3d_poly_it, all_obbs_as_one_polyh_the_points,orig_obbs,nof_the_facet_vertices,the_facet_vertices ,child1_rank_number,last_bbs_pile1,cij_by_facet,evects1,momentH1,n1,faces1,facet_moments,facet_area,is_robot,is_convex);
  if(n2>1)
    split_and_recurse_for_obbs(m3d_poly_it, all_obbs_as_one_polyh_the_points,orig_obbs,nof_the_facet_vertices,the_facet_vertices ,child2_rank_number,last_bbs_pile2,cij_by_facet,evects2,momentH2,n2,faces2,facet_moments,facet_area,is_robot,is_convex);

  /* clean up */
  if(n1 != 0) MY_FREE(faces1,int,n1);
  if(n2 != 0) MY_FREE(faces2,int,n2);

}

/* **************************************************************************** */
/* the function enclose_obbs_with_obb_tree() does:                              */
/*   compute RAPID data for each of the boxes at [hierheight-1]                 */
/*   compute the OBB of all boxes at [hierheight-1]                             */
/*   put the OBB away (cf. put_obb_away() )                                     */
/*   call recursively (cf. split_and_recurse() )                                */
/*   make joint with the OBB(-tree) already computed on polyhedron in the boxes */
/* **************************************************************************** */
int enclose_obbs_with_obb_tree(int group_nr_in_all_bb, int nof_orig_obbs, kcd_bb_p *orig_obbs, kcd_vector3 *the_points,int nb_points,kcd_index_p *the_facet_vertices)
{
  void *m3d_poly_it = NULL;
  int *the_vertices = NULL;
  int *nof_the_facet_vertices = NULL;
  int is_robot = FALSE;
  int is_convex = TRUE;
  double box_x,box_y,box_z;
/*  int *nof_boxes = NULL;*/
  int *to_recurse_facets = NULL;
  kcd_vector3 momentH;
  kcd_vector3 center;
/*  kcd_vector3 placed_center;*/
  kcd_vector3 evects[3];
  kcd_vector3 trans_evects1[3];
  /*kcd_vector3 orient[3];*/
  kcd_vector3 Teller,avec,total_teller;
  tricd_triangulation *tricd_decomp = NULL;
  int i,j,facet_nr,nr_triangles = 0,nr_facet_vertices,face_num,ii,jj;// set to zero to avoid warnings
/*  int ok; */
  double total_surface_area = 0.0;
  double *total_facet_area = NULL;
  double_p *triangle_areas = NULL;
  kcd_vector3 **triangle_moments = NULL;
  kcd_vector3 *facet_moments = NULL;
  kcd_matrix3 cij,cij_of_triangle;
  kcd_matrix3 *cij_of_facet;
  kcd_matrix3 cov_matrix;
  int parent_rank_number = 0;

  if(nof_orig_obbs > 0)
    {
      /* init */
      init_vector_zero(total_teller);
      /* triangulate polygon */
      face_num = nof_orig_obbs;
      nof_the_facet_vertices = MY_ALLOC(int,face_num);
      for(i=0;i<face_num;i++)
	{
	  nof_the_facet_vertices[i] = 8;
	}
      facet_moments = MY_ALLOC(kcd_vector3,face_num);
      cij_of_facet = MY_ALLOC(kcd_matrix3,face_num);
      total_facet_area = MY_ALLOC(double,face_num);
      tricd_decomp = MY_ALLOC(tricd_triangulation,face_num);
      triangle_areas = MY_ALLOC(double_p,face_num);
      triangle_moments = MY_ALLOC(pkcd_vector3,face_num);
      init_matrix_zero(cij);  
      /* prepare for recursive call */
      to_recurse_facets = MY_ALLOC(int,face_num);
      the_vertices = MY_ALLOC(int,8*face_num);
      for(j=0;j<face_num;j++)
	{
	  to_recurse_facets[j]=j;
	  for(i=0;i<8;i++)
	    {
	      the_vertices[j*8+i]=j*8+i+1;
	    }
	}

      /* for each facet: */      
      for(facet_nr = 0;facet_nr<face_num;facet_nr++) 
	{      
	  total_facet_area[facet_nr] = 0.0;
	  /* compute number of triangles */
	  nr_facet_vertices = 8;
	  nr_triangles = 12; /* =/= nr_facet_vertices - 2 */
	  /* allocate space for the triangles, in tricd_decomp */
	  tricd_decomp[facet_nr] = MY_ALLOC(tricd_triangle,nr_triangles);
	  triangle_areas[facet_nr] = MY_ALLOC(double,nr_triangles);
	  triangle_moments[facet_nr] = MY_ALLOC(kcd_vector3,nr_triangles);
	  
	  /* triangulate facet: 12 triangles */
	  /* triangle number 0 (vertices 0 1 2) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][0][0][0] = the_points[the_facet_vertices[facet_nr][0]-1][0]; /* X */
	  tricd_decomp[facet_nr][0][0][1] = the_points[the_facet_vertices[facet_nr][0]-1][1]; /* Y */
	  tricd_decomp[facet_nr][0][0][2] = the_points[the_facet_vertices[facet_nr][0]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][0][1][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][0][1][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][0][1][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][0][2][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][0][2][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][0][2][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* triangle number 1 (vertices 0 2 3)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][1][0][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][1][0][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][1][0][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][1][1][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][1][1][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][1][1][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][1][2][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][1][2][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][1][2][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* triangle number 2 (vertices 1 5 6) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][2][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0]; /* X */
	  tricd_decomp[facet_nr][2][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1]; /* Y */
	  tricd_decomp[facet_nr][2][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][2][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][2][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];;
	  tricd_decomp[facet_nr][2][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][2][2][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][2][2][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][2][2][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* triangle number 3 (vertices 1 6 2)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][3][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][3][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][3][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][3][1][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][3][1][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][3][1][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][3][2][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][3][2][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][3][2][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* triangle number 4 (vertices 4 0 3) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][4][0][0] = the_points[the_facet_vertices[facet_nr][4]-1][0]; /* X */
	  tricd_decomp[facet_nr][4][0][1] = the_points[the_facet_vertices[facet_nr][4]-1][1]; /* Y */
	  tricd_decomp[facet_nr][4][0][2] = the_points[the_facet_vertices[facet_nr][4]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][4][1][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][4][1][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][4][1][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][4][2][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][4][2][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][4][2][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* triangle number 5 (vertices 4 3 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][5][0][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][5][0][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][5][0][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][5][1][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][5][1][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][5][1][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][5][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][5][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][5][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  /* triangle number 6 (vertices 1 0 5) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][6][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0]; /* X */
	  tricd_decomp[facet_nr][6][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1]; /* Y */
	  tricd_decomp[facet_nr][6][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][6][1][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][6][1][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][6][1][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][6][2][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][6][2][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][6][2][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* triangle number 7 (vertices 1 5 4)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][7][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][7][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][7][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][7][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][7][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][7][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][7][2][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][7][2][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][7][2][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* triangle number 8 (vertices 6 5 4) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][8][0][0] = the_points[the_facet_vertices[facet_nr][6]-1][0]; /* X */
	  tricd_decomp[facet_nr][8][0][1] = the_points[the_facet_vertices[facet_nr][6]-1][1]; /* Y */
	  tricd_decomp[facet_nr][8][0][2] = the_points[the_facet_vertices[facet_nr][6]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][8][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][8][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][8][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][8][2][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][8][2][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][8][2][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* triangle number 9 (vertices 6 4 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][9][0][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][9][0][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][9][0][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][9][1][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][9][1][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][9][1][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][9][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][9][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][9][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  /* triangle number 10 (vertices 3 2 6) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][10][0][0] = the_points[the_facet_vertices[facet_nr][3]-1][0]; /* X */
	  tricd_decomp[facet_nr][10][0][1] = the_points[the_facet_vertices[facet_nr][3]-1][1]; /* Y */
	  tricd_decomp[facet_nr][10][0][2] = the_points[the_facet_vertices[facet_nr][3]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][10][1][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][10][1][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][10][1][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][10][2][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][10][2][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][10][2][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* triangle number 11 (vertices 3 6 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][11][0][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][11][0][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][11][0][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][11][1][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][11][1][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][11][1][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][11][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][11][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][11][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  
	  /* compute areas of the triangles and their moments and
	     total surface area of convex polyhedron */
	  init_vector_zero(Teller);
	  init_matrix_zero(cij_of_facet[facet_nr]);
	  for(i=0;i<nr_triangles;i++)
	    {
	      init_matrix_zero(cij_of_triangle);
	      triangle_areas[facet_nr][i]=
		compute_tri_area(tricd_decomp[facet_nr][i]);
	      compute_triangle_moment_and_cij(tricd_decomp[facet_nr][i],
					      triangle_moments[facet_nr][i],
					      triangle_areas[facet_nr][i],
					      cij_of_triangle);
	      total_facet_area[facet_nr] += triangle_areas[facet_nr][i];
	      kcd_mat3Add(cij_of_facet[facet_nr],cij_of_triangle,cij_of_facet[facet_nr]);
	      kcd_vectScale(triangle_moments[facet_nr][i], avec,
			    triangle_areas[facet_nr][i]);
	      kcd_vectAdd(Teller,avec,Teller);
	    }
	  kcd_vectCopy(Teller,facet_moments[facet_nr]);
	  kcd_mat3Add(cij,cij_of_facet[facet_nr],cij);
	  kcd_vectAdd(total_teller,facet_moments[facet_nr],total_teller); 
	  total_surface_area += total_facet_area[facet_nr];
	}

      /* compute accumulated moment */
      if(EQ(total_surface_area,0.0))
	{
	  kcd_vectCopy(total_teller,momentH);
	}
      else
	{
	  kcd_vectScale(total_teller,momentH,1.0/total_surface_area);
	}
      kcd_get_covariance(total_teller,total_surface_area,cij,cov_matrix);
      kcd_get_eigenvalues(evects,cov_matrix);

      /* put orientation right w.r.t. work frame */      
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  trans_evects1[ii][jj] = evects[jj][ii];
      kcd_get_obb_shape(&box_x, &box_y, &box_z, &center, the_points, nb_points, trans_evects1); 
      /* put obb away */
      all_bbs[group_nr_in_all_bb]->d[0] = box_x;
      all_bbs[group_nr_in_all_bb]->d[1] = box_y;
      all_bbs[group_nr_in_all_bb]->d[2] = box_z;
      all_bbs[group_nr_in_all_bb]->volume = 8.0 * box_x * box_y * box_z;
      all_bbs[group_nr_in_all_bb]->center[0] = center[0];
      all_bbs[group_nr_in_all_bb]->center[1] = center[1];
      all_bbs[group_nr_in_all_bb]->center[2] = center[2]; 
      all_bbs[group_nr_in_all_bb]->is_robot_prim = is_robot; 
      kcd_vectCopy(trans_evects1[0],all_bbs[group_nr_in_all_bb]->eigenv[0]);
      kcd_vectCopy(trans_evects1[1],all_bbs[group_nr_in_all_bb]->eigenv[1]);
      kcd_vectCopy(trans_evects1[2],all_bbs[group_nr_in_all_bb]->eigenv[2]);
      
      /* continue recursively */
      split_and_recurse_for_obbs(m3d_poly_it, the_points,orig_obbs,nof_the_facet_vertices,the_facet_vertices,parent_rank_number,all_bbs[group_nr_in_all_bb],cij_of_facet,evects,momentH,face_num,to_recurse_facets,facet_moments,total_facet_area,is_robot,is_convex);

      /* clean up */
      for(facet_nr = 0; facet_nr < face_num; facet_nr++)
	{
	  MY_FREE(tricd_decomp[facet_nr],tricd_triangle,nr_triangles);
	  MY_FREE(triangle_areas[facet_nr],double,nr_triangles);
	  MY_FREE(triangle_moments[facet_nr],kcd_vector3,nr_triangles);
	}
      MY_FREE(facet_moments,kcd_vector3,face_num);
      MY_FREE(cij_of_facet,kcd_matrix3,face_num);
      MY_FREE(total_facet_area,double,face_num);
      MY_FREE(tricd_decomp,tricd_triangulation,face_num);
      MY_FREE(triangle_areas,double_p,face_num);
      MY_FREE(triangle_moments,pkcd_vector3,face_num);
      MY_FREE(to_recurse_facets,int,face_num);
      MY_FREE(the_vertices,int,8*face_num);
      /* clean up local stuff */
      MY_FREE(nof_the_facet_vertices,int,face_num);
    }


  return parent_rank_number;
}

/* similar to enclose_obbs_with_obb_tree, but just for 1 robot body */
/* group_nr_in_all_bb == number in all_bb where to find the parent obb */
void enclose_body_obbs_with_obb_tree(int group_nr_in_all_bb,int nof_orig_obbs, kcd_bb_p *orig_obbs, kcd_vector3 *the_points, int nb_points, kcd_index_p *the_facet_vertices)
{
  void *m3d_poly_it = NULL;
  int *the_vertices = NULL;
  int *nof_the_facet_vertices = NULL;
/*   kcd_index_p *the_facet_vertices = NULL; */
  int is_robot = TRUE;
  int is_convex = TRUE;
  double box_x,box_y,box_z;
/*  int *nof_boxes = NULL;*/
  int *to_recurse_facets = NULL;
  kcd_vector3 momentH;
  kcd_vector3 center;
/*  kcd_vector3 placed_center;*/
  kcd_vector3 evects[3];
  kcd_vector3 trans_evects1[3];
 /* kcd_vector3 orient[3];*/
  kcd_vector3 Teller,avec,total_teller;
  tricd_triangulation *tricd_decomp = NULL;
  int i,j,facet_nr,nr_triangles=0,nr_facet_vertices,face_num=0,ii,jj; // set to zero to avoid warnings
  double total_surface_area = 0.0;
  double *total_facet_area = NULL;
  double_p *triangle_areas = NULL;
  kcd_vector3 **triangle_moments = NULL;
  kcd_vector3 *facet_moments = NULL;
  kcd_matrix3 cij,cij_of_triangle;
  kcd_matrix3 *cij_of_facet;
  kcd_matrix3 cov_matrix;
  int parent_rank_number = 0;

  if(nof_orig_obbs > 0)
    {
      /* init */
      init_vector_zero(total_teller);
      /* triangulate polygon */
      face_num = nof_orig_obbs;
      nof_the_facet_vertices = MY_ALLOC(int,face_num);
      for(i=0;i<face_num;i++)
	{
	  nof_the_facet_vertices[i] = 8;
	}
      
      facet_moments = MY_ALLOC(kcd_vector3,face_num);
      cij_of_facet = MY_ALLOC(kcd_matrix3,face_num);
      total_facet_area = MY_ALLOC(double,face_num);
      tricd_decomp = MY_ALLOC(tricd_triangulation,face_num);
      triangle_areas = MY_ALLOC(double_p,face_num);
      triangle_moments = MY_ALLOC(pkcd_vector3,face_num);
      init_matrix_zero(cij);  
      /* prepare for recursive call */
      to_recurse_facets = MY_ALLOC(int,face_num);
      the_vertices = MY_ALLOC(int,8*face_num);
      for(j=0;j<face_num;j++)
	{
	  to_recurse_facets[j]=j;
	  for(i=0;i<8;i++)
	    {
	      the_vertices[j*8+i]=j*8+i+1;
	    }
	}

      /* for each facet: */      
      for(facet_nr = 0;facet_nr<face_num;facet_nr++) 
	{      
	  total_facet_area[facet_nr] = 0.0;
	  /* compute number of triangles */
	  nr_facet_vertices = 8;
	  nr_triangles = 12; /* =/= nr_facet_vertices - 2 */
	  /* allocate space for the triangles, in tricd_decomp */
	  tricd_decomp[facet_nr] = MY_ALLOC(tricd_triangle,nr_triangles);
	  triangle_areas[facet_nr] = MY_ALLOC(double,nr_triangles);
	  triangle_moments[facet_nr] = MY_ALLOC(kcd_vector3,nr_triangles);
	  
	  /* triangulate facet: 12 triangles */
	  /* triangle number 0 (vertices 0 1 2) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][0][0][0] = the_points[the_facet_vertices[facet_nr][0]-1][0]; /* X */
	  tricd_decomp[facet_nr][0][0][1] = the_points[the_facet_vertices[facet_nr][0]-1][1]; /* Y */
	  tricd_decomp[facet_nr][0][0][2] = the_points[the_facet_vertices[facet_nr][0]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][0][1][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][0][1][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][0][1][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][0][2][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][0][2][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][0][2][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* triangle number 1 (vertices 0 2 3)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][1][0][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][1][0][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][1][0][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][1][1][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][1][1][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][1][1][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][1][2][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][1][2][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][1][2][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* triangle number 2 (vertices 1 5 6) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][2][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0]; /* X */
	  tricd_decomp[facet_nr][2][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1]; /* Y */
	  tricd_decomp[facet_nr][2][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][2][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][2][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];;
	  tricd_decomp[facet_nr][2][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][2][2][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][2][2][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][2][2][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* triangle number 3 (vertices 1 6 2)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][3][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][3][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][3][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][3][1][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][3][1][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][3][1][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][3][2][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][3][2][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][3][2][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* triangle number 4 (vertices 4 0 3) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][4][0][0] = the_points[the_facet_vertices[facet_nr][4]-1][0]; /* X */
	  tricd_decomp[facet_nr][4][0][1] = the_points[the_facet_vertices[facet_nr][4]-1][1]; /* Y */
	  tricd_decomp[facet_nr][4][0][2] = the_points[the_facet_vertices[facet_nr][4]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][4][1][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][4][1][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][4][1][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][4][2][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][4][2][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][4][2][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* triangle number 5 (vertices 4 3 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][5][0][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][5][0][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][5][0][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][5][1][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][5][1][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][5][1][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][5][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][5][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][5][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  /* triangle number 6 (vertices 1 0 5) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][6][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0]; /* X */
	  tricd_decomp[facet_nr][6][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1]; /* Y */
	  tricd_decomp[facet_nr][6][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][6][1][0] = the_points[the_facet_vertices[facet_nr][0]-1][0];
	  tricd_decomp[facet_nr][6][1][1] = the_points[the_facet_vertices[facet_nr][0]-1][1];
	  tricd_decomp[facet_nr][6][1][2] = the_points[the_facet_vertices[facet_nr][0]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][6][2][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][6][2][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][6][2][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* triangle number 7 (vertices 1 5 4)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][7][0][0] = the_points[the_facet_vertices[facet_nr][1]-1][0];
	  tricd_decomp[facet_nr][7][0][1] = the_points[the_facet_vertices[facet_nr][1]-1][1];
	  tricd_decomp[facet_nr][7][0][2] = the_points[the_facet_vertices[facet_nr][1]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][7][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][7][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][7][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][7][2][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][7][2][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][7][2][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* triangle number 8 (vertices 6 5 4) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][8][0][0] = the_points[the_facet_vertices[facet_nr][6]-1][0]; /* X */
	  tricd_decomp[facet_nr][8][0][1] = the_points[the_facet_vertices[facet_nr][6]-1][1]; /* Y */
	  tricd_decomp[facet_nr][8][0][2] = the_points[the_facet_vertices[facet_nr][6]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][8][1][0] = the_points[the_facet_vertices[facet_nr][5]-1][0];
	  tricd_decomp[facet_nr][8][1][1] = the_points[the_facet_vertices[facet_nr][5]-1][1];
	  tricd_decomp[facet_nr][8][1][2] = the_points[the_facet_vertices[facet_nr][5]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][8][2][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][8][2][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][8][2][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* triangle number 9 (vertices 6 4 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][9][0][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][9][0][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][9][0][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][9][1][0] = the_points[the_facet_vertices[facet_nr][4]-1][0];
	  tricd_decomp[facet_nr][9][1][1] = the_points[the_facet_vertices[facet_nr][4]-1][1];
	  tricd_decomp[facet_nr][9][1][2] = the_points[the_facet_vertices[facet_nr][4]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][9][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][9][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][9][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  /* triangle number 10 (vertices 3 2 6) */
	  /* vertex 0 */
	  tricd_decomp[facet_nr][10][0][0] = the_points[the_facet_vertices[facet_nr][3]-1][0]; /* X */
	  tricd_decomp[facet_nr][10][0][1] = the_points[the_facet_vertices[facet_nr][3]-1][1]; /* Y */
	  tricd_decomp[facet_nr][10][0][2] = the_points[the_facet_vertices[facet_nr][3]-1][2]; /* Z */
	  /* vertex 1 */
	  tricd_decomp[facet_nr][10][1][0] = the_points[the_facet_vertices[facet_nr][2]-1][0];
	  tricd_decomp[facet_nr][10][1][1] = the_points[the_facet_vertices[facet_nr][2]-1][1];
	  tricd_decomp[facet_nr][10][1][2] = the_points[the_facet_vertices[facet_nr][2]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][10][2][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][10][2][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][10][2][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* triangle number 11 (vertices 3 6 7)*/
	  /* vertex 0 */
	  tricd_decomp[facet_nr][11][0][0] = the_points[the_facet_vertices[facet_nr][3]-1][0];
	  tricd_decomp[facet_nr][11][0][1] = the_points[the_facet_vertices[facet_nr][3]-1][1];
	  tricd_decomp[facet_nr][11][0][2] = the_points[the_facet_vertices[facet_nr][3]-1][2];
	  /* vertex 1 */
	  tricd_decomp[facet_nr][11][1][0] = the_points[the_facet_vertices[facet_nr][6]-1][0];
	  tricd_decomp[facet_nr][11][1][1] = the_points[the_facet_vertices[facet_nr][6]-1][1];
	  tricd_decomp[facet_nr][11][1][2] = the_points[the_facet_vertices[facet_nr][6]-1][2];
	  /* vertex 2 */
	  tricd_decomp[facet_nr][11][2][0] = the_points[the_facet_vertices[facet_nr][7]-1][0];
	  tricd_decomp[facet_nr][11][2][1] = the_points[the_facet_vertices[facet_nr][7]-1][1];
	  tricd_decomp[facet_nr][11][2][2] = the_points[the_facet_vertices[facet_nr][7]-1][2];
	  
	  
	  /* compute areas of the triangles and their moments and
	     total surface area of convex polyhedron */
	  init_vector_zero(Teller);
	  init_matrix_zero(cij_of_facet[facet_nr]);
	  for(i=0;i<nr_triangles;i++)
	    {
	      init_matrix_zero(cij_of_triangle);
	      triangle_areas[facet_nr][i]=
		compute_tri_area(tricd_decomp[facet_nr][i]);
	      compute_triangle_moment_and_cij(tricd_decomp[facet_nr][i],
					      triangle_moments[facet_nr][i],
					      triangle_areas[facet_nr][i],
					      cij_of_triangle);
	      total_facet_area[facet_nr] += triangle_areas[facet_nr][i];
	      kcd_mat3Add(cij_of_facet[facet_nr],cij_of_triangle,cij_of_facet[facet_nr]);
	      kcd_vectScale(triangle_moments[facet_nr][i], avec,
			    triangle_areas[facet_nr][i]);
	      kcd_vectAdd(Teller,avec,Teller);
	    }
	  kcd_vectCopy(Teller,facet_moments[facet_nr]);
	  kcd_mat3Add(cij,cij_of_facet[facet_nr],cij);
	  kcd_vectAdd(total_teller,facet_moments[facet_nr],total_teller); 
	  total_surface_area += total_facet_area[facet_nr];
	}

      /* compute accumulated moment */
      if(EQ(total_surface_area,0.0))
	{
	  kcd_vectCopy(total_teller,momentH);
	}
      else
	{
	  kcd_vectScale(total_teller,momentH,1.0/total_surface_area);
	}
      kcd_get_covariance(total_teller,total_surface_area,cij,cov_matrix);
      kcd_get_eigenvalues(evects,cov_matrix); 
      
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  trans_evects1[ii][jj] = evects[jj][ii];

      kcd_get_obb_shape(&box_x, &box_y, &box_z, &center, the_points, nb_points, trans_evects1);

      /* put obb away */
      all_bbs[group_nr_in_all_bb]->d[0] = box_x;
      all_bbs[group_nr_in_all_bb]->d[1] = box_y;
      all_bbs[group_nr_in_all_bb]->d[2] = box_z;
      all_bbs[group_nr_in_all_bb]->volume = 8.0 * box_x * box_y * box_z;
      all_bbs[group_nr_in_all_bb]->center[0] = center[0];
      all_bbs[group_nr_in_all_bb]->center[1] = center[1];
      all_bbs[group_nr_in_all_bb]->center[2] = center[2]; 
      all_bbs[group_nr_in_all_bb]->is_robot_prim = is_robot; 
      kcd_vectCopy(trans_evects1[0],all_bbs[group_nr_in_all_bb]->eigenv[0]);
      kcd_vectCopy(trans_evects1[1],all_bbs[group_nr_in_all_bb]->eigenv[1]);
      kcd_vectCopy(trans_evects1[2],all_bbs[group_nr_in_all_bb]->eigenv[2]);
      
      /* continue recursively */
      split_and_recurse_for_obbs(m3d_poly_it, the_points,orig_obbs,nof_the_facet_vertices,the_facet_vertices,parent_rank_number,all_bbs[group_nr_in_all_bb],cij_of_facet,evects,momentH,face_num,to_recurse_facets,facet_moments,total_facet_area,is_robot,is_convex);

      /* clean up */
      for(facet_nr = 0; facet_nr < face_num; facet_nr++)
	{
	  MY_FREE(tricd_decomp[facet_nr],tricd_triangle,nr_triangles);
	  MY_FREE(triangle_areas[facet_nr],double,nr_triangles);
	  MY_FREE(triangle_moments[facet_nr],kcd_vector3,nr_triangles);
	}
      MY_FREE(facet_moments,kcd_vector3,face_num);
      MY_FREE(cij_of_facet,kcd_matrix3,face_num);
      MY_FREE(total_facet_area,double,face_num);
      MY_FREE(tricd_decomp,tricd_triangulation,face_num);
      MY_FREE(triangle_areas,double_p,face_num);
      MY_FREE(triangle_moments,pkcd_vector3,face_num);
      MY_FREE(to_recurse_facets,int,face_num);
      MY_FREE(the_vertices,int,8*face_num);
    }

  /* clean up local stuff */
  MY_FREE(nof_the_facet_vertices,int,face_num);
}
