#include "Util-pkg.h"
#include "Collision-pkg.h"


/* typedef double *double_p; */
static double initial_val_small_volume = 0.0;
static double user_defined_small_volume = 0.0;

void kcd_get_user_defined_small_volume(double *val)
{
  /* PrintInfo(("kcd: val = %f\n",user_defined_small_volume)); */
  *val = user_defined_small_volume;
}

void kcd_init_user_defined_small_volume(double val)
{
  initial_val_small_volume = val;
  user_defined_small_volume = val;
}

void kcd_reset_user_defined_small_volume()
{
  user_defined_small_volume = initial_val_small_volume;
}

void kcd_set_user_defined_small_volume(double val)
{
  user_defined_small_volume = val;
}

int kcd_volume_smaller_than_user_defined_small_volume(double kcd_volume)
{
  return (kcd_volume < user_defined_small_volume);
}

void init_matrix_zero(kcd_matrix3 m)
{
  int i,j;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      m[i][j]=0.0;
}

void init_vector_zero(kcd_vector3 m)
{
  int i;
  for(i=0;i<3;i++)
      m[i]=0.0;
}

double compute_tri_area(tricd_triangle tria)
{
  double surf = 0.0;
  kcd_vector3 c,v1,v2;

  kcd_vectSub(tria[0],tria[1],v1);
  kcd_vectSub(tria[0],tria[2],v2);
  kcd_vectXprod(v1,v2,c);
  surf = kcd_vectNorm(c) / 2.0 ;
  return surf;
}

void compute_triangle_moment_and_cij(tricd_triangle tria,
				     kcd_vector3 triangle_moment,
				     double tri_area,
				     kcd_matrix3 cij)
{
  kcd_vector3 v1,v2;

  /* moment */
  kcd_vectAdd(tria[0],tria[1],v1);
  kcd_vectAdd( v1    ,tria[2],v2);
  kcd_vectScale(v2,triangle_moment,1.0/3.0);

  /* cij */
  cij[0][0] += tri_area*(9*triangle_moment[0]*triangle_moment[0]+
	       tria[0][0]*tria[0][0]+tria[1][0]*tria[1][0]
	       +tria[2][0]*tria[2][0])/12;
  cij[0][1] += tri_area*(9*triangle_moment[0]*triangle_moment[1]+
	       tria[0][0]*tria[0][1]+tria[1][0]*tria[1][1]
	       +tria[2][0]*tria[2][1])/12;
  cij[1][1] += tri_area*(9*triangle_moment[1]*triangle_moment[1]+
	       tria[0][1]*tria[0][1]+tria[1][1]*tria[1][1]
	       +tria[2][1]*tria[2][1])/12;
  cij[0][2] += tri_area*(9*triangle_moment[0]*triangle_moment[2]+
	       tria[0][0]*tria[0][2]+tria[1][0]*tria[1][2]
	       +tria[2][0]*tria[2][2])/12;
  cij[1][2] += tri_area*(9*triangle_moment[1]*triangle_moment[2]+
	       tria[0][1]*tria[0][2]+tria[1][1]*tria[1][2]
	       +tria[2][1]*tria[2][2])/12;
  cij[2][2] += tri_area*(9*triangle_moment[2]*triangle_moment[2]+
	       tria[0][2]*tria[0][2]+tria[1][2]*tria[1][2]
	       +tria[2][2]*tria[2][2])/12;
  cij[2][1] = cij[1][2];
  cij[1][0] = cij[0][1];
  cij[2][0] = cij[0][2];
}

#define rfabs(x) ((x < 0) ? -x : x)
#define cnul(x) ((x < 0.00001)&&(x > -0.00001) ? 0.0 : x)
#define ROT(a,i,j,k,l) g=a[i][j]; h=a[k][l]; a[i][j]=g-s*(h+g*tau); a[k][l]=h+s*(g-h*tau);

int find_eigen(double vout[3][3], double dout[3], double a[3][3])
{
  int i;
  double tresh,theta,tau,t,sm,s,h,g,c;
  int nrot;
  double b[3];
  double z[3];
  double v[3][3];
  double d[3];
  
  v[0][0] = v[1][1] = v[2][2] = 1.0;
  v[0][1] = v[1][2] = v[2][0] = 0.0;
  v[0][2] = v[1][0] = v[2][1] = 0.0;
  
  b[0] = a[0][0]; d[0] = a[0][0]; z[0] = 0.0;
  b[1] = a[1][1]; d[1] = a[1][1]; z[1] = 0.0;
  b[2] = a[2][2]; d[2] = a[2][2]; z[2] = 0.0;

  nrot = 0;

  /* B Kineo Carl better initialize dout and vout */
  kcd_mat3Copy(v,vout);
  kcd_vectCopy(d,dout); 
  /* E Kineo Carl better initialize dout and vout */
  
  for(i=0; i<50; i++)
    {

      sm=0.0; sm+=fabs(a[0][1]); sm+=fabs(a[0][2]); sm+=fabs(a[1][2]);
      if (cnul(sm) == 0.0) 
	{ 
	  kcd_mat3Copy(v,vout); 
	  kcd_vectCopy(d,dout); 
 	  return i; 
	}


      
      if (i < 3) tresh=0.2*sm/(3*3); else tresh=0.0;
      
      {
	g = 100.0*rfabs(a[0][1]);  
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[1])+g==rfabs(d[1]))
	  a[0][1]=0.0;
	else if (rfabs(a[0][1])>tresh)
	  {
	    h = d[1]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][1])/h;
	    else
	      {
		theta=0.5*h/(a[0][1]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][1];
	    z[0] -= h; z[1] += h; d[0] -= h; d[1] += h;
	    a[0][1]=0.0;
	    ROT(a,0,2,1,2); ROT(v,0,0,0,1); ROT(v,1,0,1,1); ROT(v,2,0,2,1); 
	    nrot++;
	  }
      }

      {
	g = 100.0*rfabs(a[0][2]);
	if (i>3 && rfabs(d[0])+g==rfabs(d[0]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[0][2]=0.0;
	else if (rfabs(a[0][2])>tresh)
	  {
	    h = d[2]-d[0];
	    if (rfabs(h)+g == rfabs(h)) t=(a[0][2])/h;
	    else
	      {
		theta=0.5*h/(a[0][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[0][2];
	    z[0] -= h; z[2] += h; d[0] -= h; d[2] += h;
	    a[0][2]=0.0;
	    ROT(a,0,1,1,2); ROT(v,0,0,0,2); ROT(v,1,0,1,2); ROT(v,2,0,2,2); 
	    nrot++;
	  }
      }


      {
	g = 100.0*rfabs(a[1][2]);
	if (i>3 && rfabs(d[1])+g==rfabs(d[1]) && rfabs(d[2])+g==rfabs(d[2]))
	  a[1][2]=0.0;
	else if (rfabs(a[1][2])>tresh)
	  {
	    h = d[2]-d[1];
	    if (rfabs(h)+g == rfabs(h)) t=(a[1][2])/h;
	    else
	      {
		theta=0.5*h/(a[1][2]);
		t=1.0/(rfabs(theta)+sqrt(1.0+theta*theta));
		if (theta < 0.0) t = -t;
	      }
	    c=1.0/sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a[1][2];
	    z[1] -= h; z[2] += h; d[1] -= h; d[2] += h;
	    a[1][2]=0.0;
	    ROT(a,0,1,0,2); ROT(v,0,1,0,2); ROT(v,1,1,1,2); ROT(v,2,1,2,2); 
	    nrot++;
	  }
      }

      b[0] += z[0]; d[0] = b[0]; z[0] = 0.0;
      b[1] += z[1]; d[1] = b[1]; z[1] = 0.0;
      b[2] += z[2]; d[2] = b[2]; z[2] = 0.0;
      
    }

  fprintf(stderr, "eigen: too many iterations in Jacobi transform (%d).\n", i);

  return i;
}

void kcd_get_covariance(kcd_vector3 momentH,
			double surf_area,
			kcd_matrix3 accum_cij,
			kcd_matrix3 cov_matrix)
{
  int i,j;

  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      cov_matrix[i][j] = accum_cij[i][j] - momentH[i]*momentH[j] / surf_area;
}

int kcd_get_eigenvalues(kcd_matrix3 evecs,kcd_matrix3 cov_matrix)
{
  double t;
  double evals[3];
  int n;

  n = find_eigen(evecs, evals, cov_matrix);
  if (evals[2] > evals[0])
    {
      if (evals[2] > evals[1])
	{
	  /* 2 is largest, swap with column 0 */
	  t = evecs[0][2]; 
	  evecs[0][2] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][2]; 
	  evecs[1][2] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][2]; 
	  evecs[2][2] = evecs[2][0]; 
	  evecs[2][0] = t;

	}
      else
	{
	  /* 1 is largest, swap with column 0 */
	  t = evecs[0][1]; 
	  evecs[0][1] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][1]; 
	  evecs[1][1] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][1]; 
	  evecs[2][1] = evecs[2][0]; 
	  evecs[2][0] = t;

	}
    }
  else
    {
      if (evals[0] > evals[1])
	{
	  /* 0 is largest, do nothing */
	}
      else
	{
  	  /* 1 is largest */
	  t = evecs[0][1]; 
	  evecs[0][1] = evecs[0][0]; 
	  evecs[0][0] = t;
	  t = evecs[1][1]; 
	  evecs[1][1] = evecs[1][0]; 
	  evecs[1][0] = t;
	  t = evecs[2][1]; 
	  evecs[2][1] = evecs[2][0]; 
	  evecs[2][0] = t;

	}
    }

  /*  we are returning the number of iterations find_eigen took. */
  /*  too many iterations means our chosen orientation is bad.   */
  return n; 
}

void kcd_get_obb_shape_facets(double *box_x,double *box_y,double *box_z, kcd_vector3 *center,
		       kcd_vector3 *polyh_the_points,int *nof_the_facet_vertices, kcd_index_p *the_facet_vertices,int nr_facets, int *selected_faces, kcd_matrix3 evecs)
{
  int i;
  int ixmin,ixmax,iymin,iymax,izmin,izmax;
  double xval=0.0,yval=0.0,zval=0.0;
  double xmin=0.0,xmax=0.0,ymin=0.0,ymax=0.0,zmin=0.0,zmax=0.0;
  int p_it,nof_points_in_f,id_point_in_f;
  /* p3d_vector3 vp; */
  kcd_vector3  center_for_eigenframe;
  kcd_matrix3 evecs_inv; /*pr;*/

  kcd_mat3Transpose(evecs,evecs_inv); /* inverse matrix == transpose */

  /* walk through the vertices of the selected faces of polyh */
  /* i = 0 */
  id_point_in_f = the_facet_vertices[selected_faces[0]][0];
  xmin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[0]); xmax = xmin; ixmin = 0; ixmax = 0;
  ymin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[1]); ymax = ymin; iymin = 0; iymax = 0;
  zmin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[2]); zmax = zmin; izmin = 0; izmax = 0;

  /* i > 0 */
  for(i=0;i<nr_facets;i++)
    {
      nof_points_in_f = nof_the_facet_vertices[selected_faces[i]];
      for(p_it=0;p_it<nof_points_in_f;p_it++)
	{
	  id_point_in_f = the_facet_vertices[selected_faces[i]][p_it];
	  /* determine point most to the left and most to the right w.r.t. each of the directions
	     defined by evecs */
	  xval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[0]);
	  yval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[1]);
	  zval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[2]);

	  if(xval<xmin){
	    xmin = xval;ixmin=i-1;}
	  if(xval>xmax){
	    xmax = xval;ixmax=i-1;}
	  if(yval<ymin){
	    ymin = yval;iymin=i-1;}
	  if(yval>ymax){
	    ymax = yval;iymax=i-1;}
	  if(zval<zmin){
	    zmin = zval;izmin=i-1;}
	  if(zval>zmax){
	    zmax = zval;izmax=i-1;}
	}
    }


  /* compute distance along the axis defined by evecs between extreme points 
     -> box_f * 2.0 */
  *box_x = (xmax-xmin)/2.0;
  *box_y = (ymax-ymin)/2.0;
  *box_z = (zmax-zmin)/2.0;
  /* compute center of box (middle along the axis defined by evecs between extreme points)
     -> center */
  center_for_eigenframe[0] = 
    (xmax+xmin)/2.0 ; 
  center_for_eigenframe[1] = 
    (ymax+ymin)/2.0 ; 
  center_for_eigenframe[2] = 
    (zmax+zmin)/2.0  ;

  kcd_Transfo3(evecs_inv,center_for_eigenframe,(*center));
}

void kcd_get_obb_shape_facets_and_triangles(double *box_x,double *box_y,double *box_z, kcd_vector3 *center,
		       kcd_vector3 *polyh_the_points,int *nof_the_facet_vertices, kcd_index_p *the_facet_vertices,int nr_facets, int *selected_faces, kcd_matrix3 evecs)
{
  int i;
  int ixmin,ixmax,iymin,iymax,izmin,izmax;
  double xval=0.0,yval=0.0,zval=0.0;
  double xmin=0.0,xmax=0.0,ymin=0.0,ymax=0.0,zmin=0.0,zmax=0.0;
  int p_it,nof_points_in_f,id_point_in_f;
  kcd_vector3  center_for_eigenframe;
  kcd_matrix3 evecs_inv; /*pr;*/

  kcd_mat3Transpose(evecs,evecs_inv); /* inverse matrix == transpose */

  /* walk through the vertices of the selected faces  */
  /* i = 0 */
  id_point_in_f = the_facet_vertices[selected_faces[0]][0];
  xmin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[0]); xmax = xmin; ixmin = 0; ixmax = 0;
  ymin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[1]); ymax = ymin; iymin = 0; iymax = 0;
  zmin = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[2]); zmax = zmin; izmin = 0; izmax = 0;

  /* i > 0 */
  for(i=0;i<nr_facets;i++)
    {
      nof_points_in_f = nof_the_facet_vertices[selected_faces[i]];
      for(p_it=0;p_it<nof_points_in_f;p_it++)
	{
	  id_point_in_f = the_facet_vertices[selected_faces[i]][p_it];

	  /* determine point most to the left and most to the right w.r.t. each of the directions
	     defined by evecs */
	  xval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[0]);
	  yval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[1]);
	  zval = kcd_vectDotProd(polyh_the_points[id_point_in_f-1],evecs[2]);

	  if(xval<xmin){
	    xmin = xval;ixmin=i-1;}
	  if(xval>xmax){
	    xmax = xval;ixmax=i-1;}
	  if(yval<ymin){
	    ymin = yval;iymin=i-1;}
	  if(yval>ymax){
	    ymax = yval;iymax=i-1;}
	  if(zval<zmin){
	    zmin = zval;izmin=i-1;}
	  if(zval>zmax){
	    zmax = zval;izmax=i-1;}
	}
    }


  /* compute distance along the axis defined by evecs between extreme points 
     -> box_f * 2.0 */
  *box_x = (xmax-xmin)/2.0;
  *box_y = (ymax-ymin)/2.0;
  *box_z = (zmax-zmin)/2.0;
  /* compute center of box (middle along the axis defined by evecs between extreme points)
     -> center */
  center_for_eigenframe[0] = 
    (xmax+xmin)/2.0 ; 
  center_for_eigenframe[1] = 
    (ymax+ymin)/2.0 ; 
  center_for_eigenframe[2] = 
    (zmax+zmin)/2.0  ;

  kcd_Transfo3(evecs_inv,center_for_eigenframe,(*center));
}

void kcd_get_obb_shape(double *box_x,double *box_y,double *box_z, kcd_vector3 *center,
		       kcd_vector3 *polyh_the_points, int nbpts, kcd_matrix3 evecs)
{
  int i;
  int ixmin,ixmax,iymin,iymax,izmin,izmax;
  double xval=0.0,yval=0.0,zval=0.0;
  double xmin=0.0,xmax=0.0,ymin=0.0,ymax=0.0,zmin=0.0,zmax=0.0;
  kcd_vector3 center_for_eigenframe;
  kcd_matrix3 evecs_inv; /*pr;*/

  kcd_mat3Transpose(evecs,evecs_inv); /* inverse matrix == transpose */

  /* walk through the vertices of polyh */
  /* i = 0 */
  xmin = kcd_vectDotProd(polyh_the_points[0],evecs[0]); xmax = xmin; ixmin = 0; ixmax = 0;
  ymin = kcd_vectDotProd(polyh_the_points[0],evecs[1]); ymax = ymin; iymin = 0; iymax = 0;
  zmin = kcd_vectDotProd(polyh_the_points[0],evecs[2]); zmax = zmin; izmin = 0; izmax = 0;
  /* i > 0 */
  for(i=1;i<nbpts;i++)
    {
      /* determine point most to the left and most to the right w.r.t. each of the directions
	 defined by evecs */
      xval = kcd_vectDotProd(polyh_the_points[i],evecs[0]);
      yval = kcd_vectDotProd(polyh_the_points[i],evecs[1]);
      zval = kcd_vectDotProd(polyh_the_points[i],evecs[2]);
      if(xval<xmin){
	xmin = xval;ixmin=i-1;}
      else if(xval>xmax){
	xmax = xval;ixmax=i-1;}
      if(yval<ymin){
	ymin = yval;iymin=i-1;}
      else if(yval>ymax){
	ymax = yval;iymax=i-1;}
      if(zval<zmin){
	zmin = zval;izmin=i-1;}
      else if(zval>zmax){
	zmax = zval;izmax=i-1;}
    }
  /* compute distance along the axis defined by evecs between extreme points 
     -> box_f * 2.0 */
  *box_x = (xmax-xmin)/2.0;
  *box_y = (ymax-ymin)/2.0;
  *box_z = (zmax-zmin)/2.0;
  /* compute center of box (middle along the axis defined by evecs between extreme points)
     -> center */
  center_for_eigenframe[0] = 
    (xmax+xmin)/2.0 ; 
  center_for_eigenframe[1] = 
    (ymax+ymin)/2.0 ; 
  center_for_eigenframe[2] = 
    (zmax+zmin)/2.0  ;

  kcd_Transfo3(evecs_inv,center_for_eigenframe,(*center));
}



void split_and_recurse(void *m3d_poly_it,int ext_p_id, int ext_obj_id, int *nof_the_facet_vertices, kcd_index_p *the_facet_vertices, int parent_rank_number, kcd_bb_p parent_p, kcd_matrix3 *cij_by_facet, kcd_vector3 *evects,kcd_vector3 momentH,int face_num,int *recursed_facets,kcd_vector3 *facet_moments,double *facet_area,int is_robot, int is_convex)
{
  int small_volume_reached1 = FALSE, must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */
  int small_volume_reached2 = FALSE; /* Modif. Carl on volume */
  double this_bb_volume; /* Modif. Carl on volume */
  kcd_index *these_vertices = NULL;
  int nof_vertices = 0;
  int is_leaf = FALSE;
  kcd_bb_p last_bbs_pile1=NULL,last_bbs_pile2=NULL;
  double axdmp,total_facet_area1 = 0.0, total_facet_area2 = 0.0;
  kcd_vector3 Teller1,Teller2;
  int n1,n2,oldn1,oldn2,facet_nr,i,ii,jj;
  int *faces1=NULL;
  int *faces2=NULL;
  kcd_vector3 momentH1;
  kcd_vector3 momentH2;
  kcd_matrix3 cov_matrix;
  kcd_vector3 center1;
  kcd_vector3 placed_center1;
  kcd_vector3 evects1[3];
  kcd_vector3 trans_evects1[3];
  kcd_vector3 orient1[3];
  kcd_vector3 center2;
  kcd_vector3 placed_center2;
  kcd_vector3 evects2[3];
  kcd_vector3 trans_evects2[3];  
  kcd_vector3 orient2[3];
  kcd_matrix3 cij1,cij2;
  kcd_matrix4 *m3d_poly_it_pos0;
  kcd_vector3 *the_points;
  double box_x,box_y,box_z;
  int child1_rank_number =0 ,child2_rank_number =0; // put to zero to avoid warning 

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

#ifdef OBB_EVALUATION
  FILE *OBBSPLIT;
  int we_can_write = TRUE;
#endif

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
      oldn1 = n1;
      oldn2 = n2;
/*       if(n1 != 0) MY_FREE(faces1,int,n1); faces1 = NULL; */
/*       if(n2 != 0) MY_FREE(faces2,int,n2); faces2 = NULL; */
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
      /* faces1 = MY_ALLOC(int,n1); */
      faces1 = MY_REALLOC(faces1,int,oldn1,n1);
      for(i=0;i<n1;i++)
	{
	  facet_nr = recursed_facets[i];
	  faces1[i]=facet_nr;
	  kcd_vectAdd(Teller1,facet_moments[facet_nr],Teller1);
	  total_facet_area1 += facet_area[facet_nr];
	}
      n2 = face_num - n1;
      /* faces2 = MY_ALLOC(int,n2); */
      faces2 = MY_REALLOC(faces2,int,oldn2,n2);
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
      if((n2 == 1) || (n1 == 1))
	if(we_can_write) fprintf(OBBSPLIT,"*****************DISASTER*******************\n");
      if(we_can_write) fprintf(OBBSPLIT,"partitioning succeeded: n1=%i, n2=%i\n",n1,n2);
    }
#endif
  if(!EQ(total_facet_area1,0.0))
    kcd_vectScale(Teller1,momentH1,1.0/total_facet_area1);
  else
    {
      kcd_vectCopy(Teller1,momentH1);
      /* PrintInfo(("split: zero total_facet_area1\n")); */
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
      kcd_get_covariance(Teller1,total_facet_area1,cij1,cov_matrix);
      kcd_get_eigenvalues(evects1,cov_matrix); 
      /* put orientation right w.r.t. work frame */
      /* independent of is_robot: */
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  orient1[ii][jj] = evects1[jj][ii];

      if(is_robot)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_robot,&m3d_poly_it_pos0);

      kcd_TransfoVect(*m3d_poly_it_pos0,orient1[0],trans_evects1[0]); 
      kcd_TransfoVect(*m3d_poly_it_pos0,orient1[1],trans_evects1[1]); 
      kcd_TransfoVect(*m3d_poly_it_pos0,orient1[2],trans_evects1[2]); 
      /* we assume last line of interm == 0 0 0 1 */
      /* put obb away */
      kcd_get_pt_arr(m3d_poly_it,&the_points);
      if(is_convex)
	kcd_get_obb_shape_facets(&box_x, &box_y, &box_z, &center1, the_points,nof_the_facet_vertices,the_facet_vertices, n1, faces1,orient1);
      else
	kcd_get_obb_shape_facets_and_triangles(&box_x, &box_y, &box_z, &center1,the_points,nof_the_facet_vertices,the_facet_vertices, n1, faces1, orient1);
      kcd_TransfoPoint(*m3d_poly_it_pos0,center1,placed_center1);

      if(n1==1)
	{
	  small_volume_reached1 = FALSE;
	  if(is_convex)
	    {
	      nof_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,faces1[0]+1);
	      kcd_get_pt_in_f_arr(m3d_poly_it,faces1[0]+1,&these_vertices);
	    }
	  else
	    {
	      nof_vertices = nof_the_facet_vertices[faces1[0]];
	      these_vertices = the_facet_vertices[faces1[0]];
	    }
	  /* begin Modif. Carl (volume) */
	  child1_rank_number = put_obb_away(small_volume_reached1,box_x, box_y, box_z, placed_center1, trans_evects1, m3d_poly_it, ext_p_id, ext_obj_id, nof_vertices,these_vertices , OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	  /* end Modif. Carl (volume) */
	}
      else
	{  
	  /* begin modif. Carl on volume */
	  /* compute volume of OBB around faces1 */
	  this_bb_volume = 8.0 * box_x * box_y * box_z;
	  /* compare volume with user defined small volume */
	  small_volume_reached1 = ((!is_robot)||(is_robot && must_test_volume_for_robots)) ? ( this_bb_volume < user_defined_small_volume ) : FALSE;
	  /* end modif. Carl on volume */
	  child1_rank_number = put_obb_away(small_volume_reached1,box_x, box_y, box_z, placed_center1, trans_evects1, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL, OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	}
      /* end modif. */
      
      make_tree_connection(child1_rank_number,parent_p);

      last_bbs_pile1 = last_bbs_pile;
      /* :independent of is_robot */

    }
  
  /* child number 2 */
  if(n2>0)
    {      
      kcd_get_covariance(Teller2,total_facet_area2,cij2,cov_matrix);
      kcd_get_eigenvalues(evects2,cov_matrix); 
      /* independent of is_robot: */
      /* put orientation right w.r.t. work frame */
      for(ii=0;ii<3;ii++)
	for(jj=0;jj<3;jj++)
	  orient2[ii][jj] = evects2[jj][ii];
      kcd_get_pt_arr(m3d_poly_it,&the_points);
      if(is_convex)
	kcd_get_obb_shape_facets(&box_x, &box_y, &box_z, &center2, the_points,nof_the_facet_vertices,the_facet_vertices, n2, faces2,orient2);
      else
	kcd_get_obb_shape_facets_and_triangles(&box_x, &box_y, &box_z, &center2,the_points,nof_the_facet_vertices,the_facet_vertices, n2, faces2, orient2);

      /* we assume last line of interm == 0 0 0 1 */
      
      if(is_robot)
	kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
      else
	kcd_get_prim_abs_pos(m3d_poly_it,is_robot,&m3d_poly_it_pos0);

      kcd_TransfoVect(*m3d_poly_it_pos0,orient2[0],trans_evects2[0]);
      kcd_TransfoVect(*m3d_poly_it_pos0,orient2[1],trans_evects2[1]);
      kcd_TransfoVect(*m3d_poly_it_pos0,orient2[2],trans_evects2[2]);
      
      kcd_TransfoPoint(*m3d_poly_it_pos0,center2,placed_center2);
      
      if(n2==1)
	{
	  small_volume_reached2 = FALSE;
	  if(is_convex)
	    {
	      nof_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,faces2[0]+1);
	      kcd_get_pt_in_f_arr(m3d_poly_it,faces2[0]+1,&these_vertices);
	    }
	  else
	    {
	      nof_vertices = nof_the_facet_vertices[faces2[0]];
	      these_vertices = the_facet_vertices[faces2[0]];
	    }
	  /* begin Modif. Carl (volume) */
	  child2_rank_number = put_obb_away(small_volume_reached2,box_x, box_y, box_z, placed_center2, trans_evects2, m3d_poly_it, ext_p_id, ext_obj_id,nof_vertices,these_vertices, OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	  /* end Modif. Carl (volume) */
	}
      else
	{
	  /* begin modif. Carl on volume */
	  /* compute volume of OBB around faces2 */
	  this_bb_volume = 8.0 * box_x * box_y * box_z;
	  /* compare volume with user defined small volume */
	  small_volume_reached2 = ((!is_robot)||(is_robot && must_test_volume_for_robots)) ? ( this_bb_volume < user_defined_small_volume ) : FALSE;
	  /* end modif. Carl on volume */
	  child2_rank_number = put_obb_away(small_volume_reached2,box_x, box_y, box_z, placed_center2, trans_evects2, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL, OBB_SPHERICAL,is_leaf,is_robot,FALSE); 
	}
      make_tree_connection(child2_rank_number,parent_p);
      last_bbs_pile2 = last_bbs_pile;
      /* :independent of is_robot */
      
    }


#ifdef OBB_EVALUATION
  if(we_can_write) fclose(OBBSPLIT);
#endif
  /* continue recursively */
  if((n1>1)&&(!small_volume_reached1))   /* Modif. Carl on volume */
    split_and_recurse(m3d_poly_it,ext_p_id,ext_obj_id,nof_the_facet_vertices,the_facet_vertices ,child1_rank_number,last_bbs_pile1,cij_by_facet,evects1,momentH1,n1,faces1,facet_moments,facet_area,is_robot,is_convex);
  if((n2>1)&&(!small_volume_reached2))   /* Modif. Carl on volume */
    split_and_recurse(m3d_poly_it,ext_p_id,ext_obj_id,nof_the_facet_vertices,the_facet_vertices ,child2_rank_number,last_bbs_pile2,cij_by_facet,evects2,momentH2,n2,faces2,facet_moments,facet_area,is_robot,is_convex);

  /* clean up */
  if(n1 != 0) MY_FREE(faces1,int,n1);
  if(n2 != 0) MY_FREE(faces2,int,n2);
}

int enclose_convex_set_with_obb_tree(void *m3d_poly_it, int ext_p_id, int ext_obj_id, int is_robot)
{
  int *nof_the_facet_vertices = NULL;
  kcd_index_p *the_facet_vertices = NULL;
  int is_convex = TRUE;
  double box_x,box_y,box_z;
/*  int *nof_boxes = NULL;*/
  int *to_recurse_facets = NULL;
  kcd_vector3 momentH;
  kcd_vector3 center;
  kcd_vector3 placed_center;
  kcd_vector3 evects[3];
  kcd_vector3 trans_evects1[3];
  kcd_vector3 orient[3];
  kcd_vector3 Teller,avec,total_teller;
  tricd_triangulation *tricd_decomp = NULL;
  int i,j,facet_nr,nr_triangles,nr_facet_vertices,face_num,ok,ii,jj;
  double total_surface_area = 0.0;
  double *total_facet_area = NULL;
  double **triangle_areas = NULL;
  kcd_vector3 **triangle_moments = NULL;
  kcd_vector3 *facet_moments = NULL;
  kcd_matrix3 cij,cij_of_triangle;
  kcd_matrix3 *cij_of_facet;
  kcd_matrix3 cov_matrix;
  kcd_matrix4 *m3d_poly_it_pos0;
  kcd_vector3 *the_points;
  int *facet_i_pts; /* index array for points of facet; length of array = nr_facet_vertices */
  int parent_rank_number = -1;
  int must_test_volume_for_robots = FALSE, small_volume_reached;   /* Modif. Carl on volume */
  double this_bb_volume;      /* Modif. Carl on volume */

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  /* init */
  init_vector_zero(total_teller);
  /* triangulate polygon */
  face_num = kcd_get_nb_fs(m3d_poly_it);
  kcd_get_pt_arr(m3d_poly_it,&the_points);

  facet_moments = MY_ALLOC(kcd_vector3,face_num);
  cij_of_facet = MY_ALLOC(kcd_matrix3,face_num);
  total_facet_area = MY_ALLOC(double,face_num);
  tricd_decomp = MY_ALLOC(tricd_triangulation,face_num);
  triangle_areas = MY_ALLOC(double *,face_num);
  triangle_moments = MY_ALLOC(pkcd_vector3,face_num);
  init_matrix_zero(cij);  
  /* prepare for recursive call */
  to_recurse_facets = MY_ALLOC(int,face_num);
  for(j=0;j<face_num;j++)
    to_recurse_facets[j]=j;
  /* for each facet: */
  for(facet_nr = 1;facet_nr<=face_num;facet_nr++) 
    {
      total_facet_area[facet_nr-1] = 0.0;
      /* compute number of triangles */
      nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr);
      kcd_get_pt_in_f_arr(m3d_poly_it,facet_nr,&facet_i_pts);
      nr_triangles = nr_facet_vertices - 2;
      /* allocate space for the triangles, in tricd_decomp */
      tricd_decomp[facet_nr-1] = MY_ALLOC(tricd_triangle,nr_triangles);
      triangle_areas[facet_nr-1] = MY_ALLOC(double,nr_triangles);
      triangle_moments[facet_nr-1] = MY_ALLOC(kcd_vector3,nr_triangles);
      
      /* triangulate facet */
      ok = kcd_tri_angulate_facet_p(the_points,facet_i_pts,nr_facet_vertices,nr_triangles,&(tricd_decomp[facet_nr-1]));

      /* compute areas of the triangles and their moments and
	 total surface area of convex polyhedron */
      if(ok)
	{
	  init_vector_zero(Teller);
	  init_matrix_zero(cij_of_facet[facet_nr-1]);
	  for(i=0;i<nr_triangles;i++)
	    {
	      init_matrix_zero(cij_of_triangle);
	      triangle_areas[facet_nr-1][i]=
		compute_tri_area(tricd_decomp[facet_nr-1][i]);
	      compute_triangle_moment_and_cij(tricd_decomp[facet_nr-1][i],
					      triangle_moments[facet_nr-1][i],
					      triangle_areas[facet_nr-1][i],
					      cij_of_triangle);
	      total_facet_area[facet_nr-1] += triangle_areas[facet_nr-1][i];
	      kcd_mat3Add(cij_of_facet[facet_nr-1],cij_of_triangle,cij_of_facet[facet_nr-1]);
	      kcd_vectScale(triangle_moments[facet_nr-1][i], avec,
			    triangle_areas[facet_nr-1][i]);
	      kcd_vectAdd(Teller,avec,Teller);
	    }
	  kcd_vectCopy(Teller,facet_moments[facet_nr-1]);
	  kcd_mat3Add(cij,cij_of_facet[facet_nr-1],cij);
	}      
      else
	{
	  init_vector_zero(facet_moments[facet_nr-1]);
	  total_facet_area[facet_nr-1] = 0.0;
	}
      kcd_vectAdd(total_teller,facet_moments[facet_nr-1],total_teller); 
      total_surface_area += total_facet_area[facet_nr-1];
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
  /* independent of is_robot: */
  for(ii=0;ii<3;ii++)
    for(jj=0;jj<3;jj++)
      orient[ii][jj] = evects[jj][ii];

  if(is_robot)
    kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
  else
    kcd_get_prim_abs_pos(m3d_poly_it,is_robot,&m3d_poly_it_pos0);

  kcd_TransfoVect(*m3d_poly_it_pos0,orient[0],trans_evects1[0]);  /* we assume last line of interm == 0 0 0 1 */
  kcd_TransfoVect(*m3d_poly_it_pos0,orient[1],trans_evects1[1]);  /* we assume last line of interm == 0 0 0 1 */
  kcd_TransfoVect(*m3d_poly_it_pos0,orient[2],trans_evects1[2]);  /* we assume last line of interm == 0 0 0 1 */
  nof_the_facet_vertices = MY_ALLOC(int,face_num);
  the_facet_vertices = MY_ALLOC(kcd_index_p,face_num);
  for(ii=0;ii<face_num;ii++)
    {
      nof_the_facet_vertices[ii] = kcd_get_nb_pts_in_f(m3d_poly_it,ii+1);
      the_facet_vertices[ii] = MY_ALLOC(kcd_index,nof_the_facet_vertices[ii]);
      for(jj=0;jj<nof_the_facet_vertices[ii];jj++)
	the_facet_vertices[ii][jj] = kcd_get_i_pt_in_f(m3d_poly_it,ii+1,jj+1);
    }
  kcd_get_pt_arr(m3d_poly_it,&the_points);
  kcd_get_obb_shape_facets(&box_x, &box_y, &box_z, &center, the_points, nof_the_facet_vertices,
					 the_facet_vertices, face_num, to_recurse_facets, orient);
  /* begin modif. Carl on volume */
  this_bb_volume = 8.0 * box_x * box_y * box_z;   /* compute volume of OBB around faces2 */
  small_volume_reached =  ((!is_robot) || (is_robot && must_test_volume_for_robots)) ? ( this_bb_volume < user_defined_small_volume ) : FALSE;   /* compare volume with user defined small volume */
  /* end modif. Carl on volume */

  kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
  /* :independent of is_robot */

  /* put obb away */
  parent_rank_number = put_obb_away(small_volume_reached,box_x, box_y, box_z, placed_center, trans_evects1, m3d_poly_it, ext_p_id, ext_obj_id, 0, NULL, OBB_SPHERICAL,FALSE,is_robot,TRUE); 

  if(!small_volume_reached)   /* Modif. Carl on volume */
    {  /* Modif. Carl on volume */
  /* continue recursively */
#ifdef CP_IS_SOLID
#else
  /* NEW */
  split_and_recurse(m3d_poly_it,ext_p_id,ext_obj_id,nof_the_facet_vertices,the_facet_vertices,parent_rank_number,last_bbs_pile,cij_of_facet,evects,momentH,face_num,to_recurse_facets,facet_moments,total_facet_area,is_robot,is_convex);

  for(ii=0;ii<face_num;ii++)
    MY_FREE(the_facet_vertices[ii],kcd_index,nof_the_facet_vertices[ii]);
  MY_FREE(the_facet_vertices,kcd_index_p,face_num);
  MY_FREE(nof_the_facet_vertices,int,face_num);

#endif
    }  /* Modif. Carl on volume */
  /* clean up */
  for(facet_nr = 0; facet_nr < face_num; facet_nr++)
    {
      nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr+1);
      nr_triangles = nr_facet_vertices - 2;
      MY_FREE(tricd_decomp[facet_nr],tricd_triangle,nr_triangles);
      MY_FREE(triangle_areas[facet_nr],double,nr_triangles);
      MY_FREE(triangle_moments[facet_nr],kcd_vector3,nr_triangles);
    }
  MY_FREE(facet_moments,kcd_vector3,face_num);
  MY_FREE(cij_of_facet,kcd_matrix3,face_num);
  MY_FREE(total_facet_area,double,face_num);
  MY_FREE(tricd_decomp,tricd_triangulation,face_num);
  MY_FREE(triangle_areas,double *,face_num);
  MY_FREE(triangle_moments,pkcd_vector3,face_num);
  MY_FREE(to_recurse_facets,int,face_num);

  return parent_rank_number;
}


int enclose_triangles_from_concave_with_obb_tree(void *m3d_poly_it, int ext_p_id, int ext_obj_id, int is_robot)
{
  int is_convex = FALSE;
  double box_x,box_y,box_z;
/*  int *nof_boxes = NULL;*/
  int *to_recurse_facets = NULL;
  kcd_vector3 momentH;
  kcd_vector3 center;
  kcd_vector3 placed_center;
  kcd_vector3 evects[3];
  kcd_vector3 trans_evects1[3];
  kcd_vector3 orient[3];
  kcd_vector3 Teller,avec,total_teller;
  tricd_triangle *tricd_decomp = NULL;
  int i,j,facet_nr,nr_triangles,nr_facet_vertices,orig_face_num,face_num=0,ok,this_facet,ii,jj;
  double total_surface_area = 0.0;
  double *total_facet_area = NULL;
  double *triangle_areas = NULL;
  kcd_vector3 *triangle_moments = NULL;
  kcd_vector3 *facet_moments = NULL;
  output_triangle_p the_triangle_id_list = NULL;
  kcd_index_p *the_facet_vertices = NULL;
  int *nof_the_facet_vertices = NULL;
  kcd_matrix3 cij,cij_of_triangle;
  kcd_matrix3 *cij_of_facet;
  kcd_matrix3 cov_matrix;
  int parent_rank_number=-1, max_nr_facet_vertices;
  int iq,jq;
  int must_test_volume_for_robots = FALSE, small_volume_reached;   /* Modif. Carl on volume */
  double this_bb_volume;      /* Modif. Carl on volume */
  kcd_matrix4 *m3d_poly_it_pos0;
  kcd_vector3 *the_points;
  int *facet_i_pts; /* index array for points of facet; length of array = nr_facet_vertices */

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  init_vector_zero(total_teller);
  /* triangulate polygon */
  orig_face_num = kcd_get_nb_fs(m3d_poly_it);
  kcd_get_pt_arr(m3d_poly_it,&the_points);
  for(facet_nr = 1;facet_nr<=orig_face_num;facet_nr++) 
    {
      if(kcd_facet_is_convex(m3d_poly_it,facet_nr))
	{
	  face_num += 1;
	}
      else
	{
	  nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr);
	  nr_triangles = nr_facet_vertices - 2;
	  face_num += nr_triangles;
	}
    }

  to_recurse_facets = MY_ALLOC(int,face_num);
  for(j=0;j<face_num;j++)
    to_recurse_facets[j]=j;
  facet_moments = MY_ALLOC(kcd_vector3,face_num);
  the_facet_vertices = MY_ALLOC(kcd_index_p,face_num);
  nof_the_facet_vertices = MY_ALLOC(int,face_num);
  cij_of_facet = MY_ALLOC(kcd_matrix3,face_num);
  total_facet_area = MY_ALLOC(double,face_num);
  init_matrix_zero(cij);  

  max_nr_facet_vertices = 2;
  for(facet_nr = 1;facet_nr<=orig_face_num;facet_nr++) 
    {
      nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr);
      if(max_nr_facet_vertices < nr_facet_vertices)
	max_nr_facet_vertices = nr_facet_vertices;
    }
  nr_triangles = max_nr_facet_vertices - 2;
  tricd_decomp = MY_ALLOC(tricd_triangle,nr_triangles);
  triangle_areas = MY_ALLOC(double,nr_triangles);
  triangle_moments = MY_ALLOC(kcd_vector3,nr_triangles);

  /* for each facet: */
  this_facet=0;
  for(facet_nr = 1;facet_nr<=orig_face_num;facet_nr++) 
    {
      if(kcd_facet_is_convex(m3d_poly_it,facet_nr))
	{
	  /* compute number of triangles */
	  nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr);
	  kcd_get_pt_in_f_arr(m3d_poly_it,facet_nr,&facet_i_pts);
	  nof_the_facet_vertices[this_facet] = nr_facet_vertices;
	  the_facet_vertices[this_facet] = MY_ALLOC(kcd_index,nr_facet_vertices);
	  for(i=1;i<=nr_facet_vertices;i++)
	    the_facet_vertices[this_facet][i-1] = kcd_get_i_pt_in_f(m3d_poly_it,facet_nr,i);
	    
	  nr_triangles = nr_facet_vertices - 2;

	  ok = kcd_tri_angulate_facet_p(the_points,facet_i_pts,nr_facet_vertices,nr_triangles,&(tricd_decomp));
	  /* compute areas of the triangles and their moments and
	     total surface area of concave polyhedron */
	  if(ok)
	    {
	      total_facet_area[this_facet]=0.0;
	      init_vector_zero(Teller);
	      init_matrix_zero(cij_of_facet[this_facet]);  
	      for(i=0;i<nr_triangles;i++)
		{
		  init_matrix_zero(cij_of_triangle);
		  triangle_areas[i]= compute_tri_area(tricd_decomp[i]);
		  compute_triangle_moment_and_cij(tricd_decomp[i],
						  triangle_moments[i],
						  triangle_areas[i],
						  cij_of_triangle);
		  total_facet_area[this_facet] += triangle_areas[i];
		  kcd_mat3Add(cij_of_facet[this_facet],cij_of_triangle,cij_of_facet[this_facet]);
		  kcd_vectScale(triangle_moments[i], avec,
				triangle_areas[i]);
		  kcd_vectAdd(Teller,avec,Teller);
		}
	      kcd_vectCopy(Teller,facet_moments[this_facet]);
	      kcd_mat3Add(cij,cij_of_facet[this_facet],cij);
	      kcd_vectAdd(total_teller,facet_moments[this_facet],total_teller);
	      total_surface_area += total_facet_area[this_facet];
	      this_facet++;
	    }
	  else
	    {
	      init_vector_zero(facet_moments[this_facet]);
	      total_facet_area[this_facet] = 0.0;
	      this_facet++;
	    }
	}
      else
	{
	  /* compute number of triangles */
	  nr_facet_vertices = kcd_get_nb_pts_in_f(m3d_poly_it,facet_nr);
	  kcd_get_pt_in_f_arr(m3d_poly_it,facet_nr,&facet_i_pts);
	  nr_triangles = nr_facet_vertices - 2;
	  the_triangle_id_list = MY_ALLOC(output_triangle,nr_triangles);
	  /* triangulate facet */
	  for(iq=0;iq<nr_triangles;iq++)
	    {
	      for(jq=0;jq<3;jq++)
		{
		  tricd_decomp[iq][jq][0]=4.0;
		  tricd_decomp[iq][jq][1]=5.0;
		  tricd_decomp[iq][jq][2]=6.0;
		}
	    }

	  ok = kcd_tri_angulate_facet_p_i(the_points,facet_i_pts,nr_facet_vertices,nr_triangles,
	                                  &(tricd_decomp),&(the_triangle_id_list));
	  /* each triangle now becomes a facet, they replace this facet */
	  /* compute areas of the triangles and their moments and
	     total surface area of concave polyhedron */
	  if(ok)
	    {
	      for(i=0;i<nr_triangles;i++)
		{
		  init_vector_zero(Teller);
		  triangle_areas[i]=
		    compute_tri_area(tricd_decomp[i]);
		  init_matrix_zero(cij_of_facet[this_facet]);
		  compute_triangle_moment_and_cij(tricd_decomp[i],
						  triangle_moments[i],
						  triangle_areas[i],
						  cij_of_facet[this_facet]);
		  total_facet_area[this_facet] = triangle_areas[i];
		  nof_the_facet_vertices[this_facet] = 3;
		  the_facet_vertices[this_facet] = MY_ALLOC(kcd_index,3);
		  for(j=0;j<3;j++)
		    the_facet_vertices[this_facet][j] = the_triangle_id_list[i][j];
		  
		  kcd_vectScale(triangle_moments[i], avec,
				triangle_areas[i]);
		  kcd_vectAdd(Teller,avec,Teller);
		  kcd_vectCopy(Teller,facet_moments[this_facet]);
		  kcd_mat3Add(cij,cij_of_facet[this_facet],cij);
		  kcd_vectAdd(total_teller,facet_moments[this_facet],total_teller);
		  total_surface_area += total_facet_area[this_facet];
		  this_facet++;
		}
	    }      
	  else
	    {
	      for(i=0;i<nr_triangles;i++)
		{
		  init_vector_zero(facet_moments[this_facet]);
		  total_facet_area[this_facet] = 0.0;
		  this_facet++;
		}
	    }
	  MY_FREE(the_triangle_id_list,output_triangle,nr_triangles);
	}
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

  if(face_num != this_facet)
    PrintInfo(("IMPOSSIBLE, not the right number of facets for concave p\n"));
  /* we now know how many facets we really have (convex faces + triangles) */
  /* select largest eigenvalue: is first in the "evects" */
  /* put orientation right w.r.t. work frame */
  /* independent of is_robot: */
  for(ii=0;ii<3;ii++)
    for(jj=0;jj<3;jj++)
      orient[ii][jj] = evects[jj][ii];
  kcd_get_pt_arr(m3d_poly_it,&the_points);
  kcd_get_obb_shape_facets_and_triangles(&box_x, &box_y, &box_z, &center,the_points,nof_the_facet_vertices,the_facet_vertices , face_num, to_recurse_facets, orient); 
 
  this_bb_volume = 8.0 * box_x * box_y * box_z;   /* compute volume of OBB around faces2 */
  small_volume_reached = ((!is_robot) || (is_robot && must_test_volume_for_robots)) ? ( this_bb_volume < user_defined_small_volume ) : FALSE;   /* compare volume with user defined small volume */
  /* end modif. Carl on volume */

  if(is_robot)
    kcd_get_prim_rel_pos(m3d_poly_it,&m3d_poly_it_pos0);
  else
    kcd_get_prim_abs_pos(m3d_poly_it,is_robot,&m3d_poly_it_pos0);

  kcd_TransfoVect(*m3d_poly_it_pos0,orient[0],trans_evects1[0]);  /* we assume last line of interm == 0 0 0 1 */
  kcd_TransfoVect(*m3d_poly_it_pos0,orient[1],trans_evects1[1]);  /* we assume last line of interm == 0 0 0 1 */
  kcd_TransfoVect(*m3d_poly_it_pos0,orient[2],trans_evects1[2]);  /* we assume last line of interm == 0 0 0 1 */
  kcd_TransfoPoint(*m3d_poly_it_pos0,center,placed_center);
  /* :independent of is_robot */

      parent_rank_number = put_obb_away(small_volume_reached,box_x, box_y, box_z, placed_center, trans_evects1, m3d_poly_it, ext_p_id, ext_obj_id, 0,NULL, OBB_SPHERICAL,FALSE,is_robot,TRUE); 
  if(!small_volume_reached)   /* Modif. Carl on volume */
    {  /* Modif. Carl on volume */
  /* continue recursively */
      split_and_recurse(m3d_poly_it,ext_p_id,ext_obj_id,nof_the_facet_vertices,the_facet_vertices ,parent_rank_number,last_bbs_pile,cij_of_facet,evects,momentH,face_num,to_recurse_facets,facet_moments,total_facet_area,is_robot,is_convex);
    }  /* Modif. Carl on volume */
  

  /* clean up */
  for(facet_nr = 0; facet_nr < face_num; facet_nr++)
    {
      MY_FREE(the_facet_vertices[facet_nr],kcd_index,nof_the_facet_vertices[facet_nr]);
    }
  MY_FREE(facet_moments,kcd_vector3,face_num);
  MY_FREE(the_facet_vertices,kcd_index_p,face_num);
  MY_FREE(nof_the_facet_vertices,int,face_num);
  MY_FREE(cij_of_facet,kcd_matrix3,face_num);
  MY_FREE(total_facet_area,double,face_num);
  MY_FREE(tricd_decomp,tricd_triangle,max_nr_facet_vertices-2);
  MY_FREE(triangle_areas,double,max_nr_facet_vertices-2);
  MY_FREE(triangle_moments,kcd_vector3,max_nr_facet_vertices-2);
  MY_FREE(to_recurse_facets,int,face_num);
 
  return parent_rank_number;
}



