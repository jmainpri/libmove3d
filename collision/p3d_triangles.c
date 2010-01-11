#include "P3d-pkg.h"
#include "Collision-pkg.h"



/* data structures for polygon triangulation */
typedef enum
{
	OXY,
	OYZ,
	OXZ
} projection_type;

typedef struct tri_vertex
{
  double x;
  double y;  /* projected coordinates of point */
  int ref;     /* index of this vertex in facet of original polyhedron */
  int p3d_ref; /* index of this vertex in description of p3d-polyhedron */
} tri_vertex;

/* global variables for polygon triangulation */
static p3d_poly *current_polyhedron = NULL;
static int current_facet = 0;
static projection_type projection = OXY;
static tri_vertex *tri_vertices = NULL;
static int nr_vertices = 0;
static output_triangle *list_of_triangles = NULL;

/* Triangulation of a simple polygon */
/* input :  p3d_polyhedron facet, oriented in counter clockwise order */
int tri_get_a_triangle_vertex(int triangle_nr,int vertex_nr)
{
  return list_of_triangles[triangle_nr][vertex_nr];
}

/* returns the "point_nr"-th point of facet "current_facet" 
   of polyhedron "current_polyhedron" */
static void tri_get_point(int point_nr, double *x, double *y, double *z)
{
  poly_get_point_in_pos_in_face(current_polyhedron->poly,current_facet,point_nr+1,x,y,z);
}

static int left(int poly_is_ccw,
	double A,
	double B,
	double C,
	double x,
	double y)
{
	if((double)poly_is_ccw*(A*x+B*y+C) > -FUZZ)
		return TRUE;
	else
		return FALSE;
}

static void tri_clean()
{
  if(tri_vertices)
    free(tri_vertices);
  tri_vertices = NULL;
  nr_vertices = 0;
  current_polyhedron = NULL;
  current_facet = 0;
  projection = OXY;
}

static int tri_project_facet(int nr_points)
{
  int poly_is_ccw = 1; /* == 1 if ccw, == -1 if cw */
  double A=0.0,B=0.0,C=0.0;
  int i;
  double orig_x, orig_y, orig_z;
  poly_plane face_plane;
  double area = 0.0, x,y;

  if(tri_vertices)
    free(tri_vertices);
  tri_vertices = NULL;
  nr_vertices = nr_points;
  tri_vertices = (tri_vertex *)malloc(sizeof(tri_vertex)*nr_vertices);

  /* plane through facet: A.X + B.Y + C.Z + D = 0, D not needed below */
  if(poly_get_plane(current_polyhedron->poly,current_facet,&face_plane))
    {
      A = face_plane.normale[0];
      B = face_plane.normale[1];
      C = face_plane.normale[2];
/*       PrintInfo(("poly_get_plane gave: %f,%f,%f\n",face_plane.normale[0],
	     face_plane.normale[1],face_plane.normale[2])); */
/*       PrintInfo(("poly_get_plane gave: %f,%f,%f\n",A,B,C)); */
    }
  else
    { A=B=C=0.0;/*error*/
    PrintInfo(("ERROR in tri_project_facet\n"));}
  /* determine the best projection to use */
  if(fabs(A) > fabs(B))
    if(fabs(A) > fabs(C))
      projection=OYZ;
    else
      projection=OXY;
  else
    if(fabs(B) > fabs(C))
      projection=OXZ;
    else
      projection=OXY;

  /* project all vertices of current facet according to best projection */
  switch(projection){
  case OXY: for(i=0;i<nr_vertices;i++)
    {
      tri_vertices[i].ref = i+1;
      tri_get_point(i,&orig_x,&orig_y,&orig_z);
      tri_vertices[i].x = orig_x;
      tri_vertices[i].y = orig_y;
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      tri_vertices[i].p3d_ref = 
	poly_get_index_point_in_face(current_polyhedron->poly,current_facet,i+1);
    }
    break;
  case OXZ: for(i=0;i<nr_vertices;i++)
    {
      tri_vertices[i].ref = i+1;
      tri_get_point(i,&orig_x,&orig_y,&orig_z);
      tri_vertices[i].x = orig_x;
      tri_vertices[i].y = orig_z;
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      tri_vertices[i].p3d_ref = 
	poly_get_index_point_in_face(current_polyhedron->poly,current_facet,i+1);
    }
    break;
  case OYZ: for(i=0;i<nr_vertices;i++)
    {
      tri_vertices[i].ref = i+1;
      tri_get_point(i,&orig_x,&orig_y,&orig_z);
      tri_vertices[i].x = orig_y;
      tri_vertices[i].y = orig_z;
      /* keep track of index of vertex in p3d-polyhedron (hack) */
      tri_vertices[i].p3d_ref = 
	poly_get_index_point_in_face(current_polyhedron->poly,current_facet,i+1);
    }
    break;
  }
  /* compute twice the area of the projected polygon */
  area = 0.0;
  x = tri_vertices[0].x;
  y = tri_vertices[0].y;
  for(i=2;i<nr_vertices;i++)
    area +=   (tri_vertices[i-1].x - x) 
            * (tri_vertices[i].y   - y)
            - (tri_vertices[i-1].y - y) 
            * (tri_vertices[i].x   - x);
  if(area < 0.0)
    poly_is_ccw = -1; /* else: poly_is_ccw = 1 */
  return poly_is_ccw;
}

static double tri_twice_the_triangle_area(int vnr_1,int vnr_2,int vnr_3)
{
  return (tri_vertices[vnr_2].x - tri_vertices[vnr_1].x)
        *(tri_vertices[vnr_3].y - tri_vertices[vnr_1].y) - 
         (tri_vertices[vnr_2].y - tri_vertices[vnr_1].y)
        *(tri_vertices[vnr_3].x - tri_vertices[vnr_1].x);
}

static int convex(int vnr_1, int vnr_2, int vnr_3, int poly_is_ccw)
{
  double t;
  int r;

  t = poly_is_ccw * tri_twice_the_triangle_area(vnr_1,vnr_2,vnr_3);
  if (t > FUZZ)
    r=1;
  else if (t < -FUZZ)
    r=0;
  else
    r=-1;
  /* PrintInfo(("convex: return value = %i\n",r));  */
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
      ba.A=tri_vertices[vnr_3].y - tri_vertices[vnr_1].y;
      ba.B=tri_vertices[vnr_1].x - tri_vertices[vnr_3].x;
      ba.C= -ba.A*tri_vertices[vnr_1].x - ba.B*tri_vertices[vnr_1].y;
      ac.A=tri_vertices[vnr_1].y - tri_vertices[vnr_2].y;
      ac.B=tri_vertices[vnr_2].x - tri_vertices[vnr_1].x;
      ac.C= -ac.A*tri_vertices[vnr_2].x - ac.B*tri_vertices[vnr_2].y;
      cb.A=tri_vertices[vnr_2].y - tri_vertices[vnr_3].y;
      cb.B=tri_vertices[vnr_3].x - tri_vertices[vnr_2].x;
      cb.C= -cb.A*tri_vertices[vnr_3].x - cb.B*tri_vertices[vnr_3].y;      
      for(i=MOD(vnr_3+1,nr_vertices);(res)&&(MOD(i,nr_vertices)!=vnr_1);i++)
	{ 
	  im = MOD(i,nr_vertices);
	  if(!((tri_vertices[vnr_1].p3d_ref == tri_vertices[im].p3d_ref) ||
	     (tri_vertices[vnr_2].p3d_ref == tri_vertices[im].p3d_ref) ||
	     (tri_vertices[vnr_3].p3d_ref == tri_vertices[im].p3d_ref) ) )
	    {
	      x = tri_vertices[im].x;
	      y = tri_vertices[im].y;
	      if(left(poly_is_ccw,ba.A,ba.B,ba.C,x,y) &&
		 left(poly_is_ccw,ac.A,ac.B,ac.C,x,y) &&
		 left(poly_is_ccw,cb.A,cb.B,cb.C,x,y))
		res = FALSE;
	    }
	  else
	    {
	      /* else: hack for non-simple polyhedral facets 
	      if(tri_vertices[vnr_1].p3d_ref == tri_vertices[i].p3d_ref)
		res = !left(poly_is_ccw,cb.A,cb.B,cb.C,x,y);
	      if(tri_vertices[vnr_2].p3d_ref == tri_vertices[i].p3d_ref)
		res = !left(poly_is_ccw,ba.A,ba.B,ba.C,x,y);
	      if(tri_vertices[vnr_3].p3d_ref == tri_vertices[i].p3d_ref)
		res = !left(poly_is_ccw,ac.A,ac.B,ac.C,x,y);*/
	    }
	}
    }

  /* if(res)
     PrintInfo(("diagonal: TRUE\n"));
     else
     PrintInfo(("diagonal: FALSE\n")); */
  return res;
}

int tri_angulate_facet(p3d_poly *m3d_polyh, int fnum, int nr_triangles,
			tricd_triangulation *tricd_decomp_p)
{
  int ok = FALSE;
  output_triangle_p *triangle_id_list_p = NULL;

  ok = tri_angulate_facet2(m3d_polyh, fnum,  nr_triangles,
		      tricd_decomp_p, triangle_id_list_p, FALSE);
  return ok;
}

int tri_angulate_facet1(p3d_poly *m3d_polyh, int fnum, int nr_triangles,
			tricd_triangulation *tricd_decomp_p,output_triangle_p *triangle_id_list_p)
{
  int ok = FALSE;

  ok = tri_angulate_facet2(m3d_polyh, fnum,  nr_triangles,
		      tricd_decomp_p, triangle_id_list_p, TRUE);

  return ok;
}

/* input:  polyhedron facet (a polygon) as described in Move3D */
/* output: list of triangles defined by the coordinates of their vertices */
int tri_angulate_facet2(p3d_poly *m3d_polyh, int fnum, int nr_triangles,
			tricd_triangulation *tricd_decomp_p, 
			output_triangle_p *triangle_id_list_p,int fill_triangle_id_list)
{
  int nof_triangles,i,j,triangle_id = 0,ok;
  double x,y,z;
  

  ok = tri_angulate_simple_polygon(m3d_polyh,fnum,&nof_triangles);
  if(nof_triangles != nr_triangles)
    PrintInfo(("WAGRHEGYU\n"));
if(ok)
    {
      if(fill_triangle_id_list)
	{
	  /* transform to list of triangle vertices */
	  for(i=0;i<nr_triangles;i++)
	    {
	      for(j=0;j<3;j++)
		{
		  triangle_id = tri_get_a_triangle_vertex(i,j);
		  /* (*triangle_id_list_p)[i][j] = triangle_id; */
		  (*triangle_id_list_p)[i][j] = p3d_get_index_point_in_face(m3d_polyh->poly,fnum,triangle_id);
		  /* PrintInfo(("triangle_id = %i,other=%i \n",triangle_id,m3d_polyh->poly->the_faces[fnum-1].the_indexs_points[triangle_id-1])); */

		  poly_get_point_in_pos_in_face(m3d_polyh->poly,
						fnum,triangle_id,&x,&y,&z);

		  (*tricd_decomp_p)[i][j][0]=x;
		  (*tricd_decomp_p)[i][j][1]=y;
		  (*tricd_decomp_p)[i][j][2]=z;
		}
	    }
	}
      else
	{
	  for(i=0;i<nr_triangles;i++)
	    {
	      for(j=0;j<3;j++)
		{
		  triangle_id = tri_get_a_triangle_vertex(i,j);
		  poly_get_point_in_pos_in_face(m3d_polyh->poly,
						fnum,triangle_id,&x,&y,&z);
		  (*tricd_decomp_p)[i][j][0]=x;
		  (*tricd_decomp_p)[i][j][1]=y;
		  (*tricd_decomp_p)[i][j][2]=z;
		}
	    }
	}
    }
  return ok;
}

static void tri_add_triangle(int vnr_1,int vnr_2,int vnr_3, 
		      int *nr_triangles)
{
  int new_tri = *nr_triangles;

  (*nr_triangles)++;
  list_of_triangles = (output_triangle *)realloc(list_of_triangles,
			    sizeof(output_triangle)*(*nr_triangles));
  list_of_triangles[new_tri][0] = tri_vertices[vnr_1].ref;
  list_of_triangles[new_tri][1] = tri_vertices[vnr_2].ref;
  list_of_triangles[new_tri][2] = tri_vertices[vnr_3].ref;
  /* PrintInfo(("new triangle added: %i,%i,%i\n",list_of_triangles[new_tri][0],
     list_of_triangles[new_tri][1],list_of_triangles[new_tri][2])); */
}

static void clip_ear(int vnr,int *nr_triangles)
{
  int i;

  /* make a triangle */
  tri_add_triangle(MOD(vnr-1,nr_vertices),vnr,MOD(vnr+1,nr_vertices),nr_triangles);
  /* eliminate vertex "vnr" from the polygon */ 
  for(i=vnr+1;i<nr_vertices;i++)
    {
      tri_vertices[i-1].ref = tri_vertices[i].ref;
      tri_vertices[i-1].p3d_ref = tri_vertices[i].p3d_ref;
      tri_vertices[i-1].x = tri_vertices[i].x;
      tri_vertices[i-1].y = tri_vertices[i].y;
    }
  nr_vertices--;
}

/* triangulates facet "facet_nr" of polyhedron "m3d_polyh" in "*nr_triangles"
   triangles listed afterwards is "*triangles" */
int tri_angulate_simple_polygon(p3d_poly *m3d_polyh, int facet_nr, 
				 int *nr_triangles)
{
  int point_it=0;
  int poly_is_ccw = 0;
  int nof_points = poly_get_nb_points_in_face(m3d_polyh->poly,facet_nr);
  /* PrintInfo(("facet nr: %i\n",facet_nr)); */
  if(list_of_triangles)
    free(list_of_triangles);
  list_of_triangles=NULL;
  (*nr_triangles) = 0;
  current_polyhedron = m3d_polyh;
  current_facet = facet_nr;
  poly_is_ccw = tri_project_facet(nof_points);

  if(poly_is_ccw != 0)
    {
      while(nof_points > 3)
	{
	  while(!diagonal(MOD(point_it,nof_points),MOD(point_it+2,nof_points),
			  poly_is_ccw))
	    {
	      ++point_it;
	    }
	  clip_ear(MOD(point_it+1,nof_points),nr_triangles);
	  --nof_points;
	  point_it=0;
	}
      /* and the last triangle is... */
      tri_add_triangle(MOD(point_it,nof_points),MOD(point_it+1,nof_points),
		       MOD(point_it+2,nof_points),nr_triangles);
      /* clean up */
      tri_clean();
      return TRUE;
    }
  else
    {
      return FALSE;
    }
}

