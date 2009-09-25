
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <stdio.h>

//! Default gpFace constructor (dimension= 2)
gpFace::gpFace()
{
  _dimension= 2;
  _v.resize(_dimension);
  _normal.resize(_dimension);
  _id= 0;
  _offset= 0.0;
  _toporient= true; 
}

//! \param dimension desired dimension of the face
gpFace::gpFace(unsigned int dimension)
{
  if( dimension < 2)
  { 
    printf("%s: %d:  gpFace::gpFace(unsigned int): face dimension must be > 1.\n",__FILE__,__LINE__);
    dimension= 2; 
  }

  _dimension= dimension;
  _v.resize(_dimension);
  _normal.resize(_dimension);
  _id= 0;
  _offset= 0.0;
  _toporient= true; 
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return the i-th element of the vertex index array of the face
unsigned int gpFace::operator [] (unsigned int i) const
{
  if(i > _dimension-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's dimension (%d).\n",__FILE__,__LINE__,i,_dimension);
    return 0;
  }

  return _v[i];
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return a reference to the i-th element of the vertex index array of the face
unsigned int& gpFace::operator [] (unsigned int i)
{
  if(i > _dimension-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's dimension (%d).\n",__FILE__,__LINE__,i,_dimension);
    return _v[0];
  }

  return _v[i];
}

//! Resizes the face.
//! \warning private member. It should not be accessed from outside.
//! \param dim the new dimension
//! \return 1 in case of success, 0 otherwise
int gpFace::resize(unsigned int dim)
{
  if(dim < 2)
  {
    printf("%s: %d: gpFace::resize(unsigned int): face dimension must be >1.\n",__FILE__,__LINE__);
    return 0;
  }

  _dimension= dim;
  _v.resize(_dimension);
  _normal.resize(_dimension);

  return 1;
}


//! Default constructor of the class gpConvexHull.
//! All is void.
gpConvexHull::gpConvexHull()
{
 _dimension= 0;
 _up_to_date= false;
 _largest_ball_radius= 0.0;
 _vertex_list= NULL;
 _facet_list= NULL;
}


gpConvexHull::~gpConvexHull()
{
  _points.clear();
  hull_vertices.clear();
  hull_faces.clear();
}



//! Computes the convex hull of the point set stored in the calling gpConvexHull variable.
//! \return 1 in case of success, 0 otherwise.
//! NB: this function relies on Qhull library's functions and frees the memory used by Qhull once
//! the convex hull computation is done (TODO: check this last point (memory management).
int gpConvexHull::compute()
{
  if(_dimension < 2)
  {
    printf("%s: %d: gpConvexHull::compute(): points dimension is must be >= 2.\n",__FILE__,__LINE__);
    return 0;
  }
  if(_points.size() < _dimension+1)
  {
    printf("%s: %d: gpConvexHull::compute(): there must be at least %d input points to compute the convex hull (dimension= %d).\n",__FILE__,__LINE__,_dimension+1, _dimension);
    return 0;
  }
  
  bool contains_origin= false;
  unsigned int i, j, cntV= 0, cntF= 0;
  double offset= 0, min_offset=-1.0;
  unsigned int numpoints;   // number of points
  coordT *point_array;      // array of coordinates for each point
  boolT ismalloc;           // True if qhull should free points in qh_freeqhull() or reallocation
  char flags[]= "qhull s -Qt"; // option flags for qhull, see qh_opt.htm
                               // OPTION 'Qt' is mandatory because we need the output facets to be simplicial
  FILE *outfile= stdout;    // output from qh_produce_output()			
	                    // use NULL to skip qh_produce_output() 
  outfile= NULL;
  FILE *errfile= stderr;    // error messages from qhull code
  int exitcode;             // 0 if no error from qhull
  vertexT *vertex, **vertexp; //vertex is used by the macro FORALLvertices, vertexp is used by the macro FOREACHvertex
  facetT *facet;  // facet is used by the macro FORALLfacets
  int curlong, totlong;	  // memory remaining after qh_memfreeshort

  // initialize numpoints, point_array[], ismalloc here
  numpoints= _points.size();
  point_array= new coordT[_dimension*numpoints];
  for(i=0; i<numpoints; i++)
  {
    if(_points[i].size()!=_dimension)
    {
      printf("%s: %d: gpConvexHull::compute(): an input point has an incorrect dimension (dimension is must be >= 2%d instead of %d).\n",__FILE__,__LINE__,_points[i].size(),_dimension);
      return 0;
    }
    for(j=0; j<_dimension; j++)
    {
      point_array[3*i+j]= _points[i][j];
    }
  }
  ismalloc= True;

  exitcode= qh_new_qhull(_dimension, numpoints, point_array, ismalloc, flags, outfile, errfile);

  contains_origin= true;
  min_offset= -1.0;
  if(!exitcode)  // if no error 
  {
     // get the hull vertices:
     hull_vertices.resize(qh_qh.num_vertices);
     cntV= 0;
     FORALLvertices
     {
       hull_vertices[cntV]= qh_pointid(vertex->point);
       cntV++;
     }

     // now, get the hull faces:
     hull_faces.resize(qh_qh.num_facets);
     cntF= 0;
     FORALLfacets
     {
        hull_faces[cntF].resize(_dimension);

        cntV= 0;
        FOREACHvertex_(facet->vertices)
        {
          hull_faces[cntF][cntV]=  qh_pointid(vertex->point);
          cntV++;
        } 

        for(i=0; i<_dimension; i++)
        {
          hull_faces[cntF]._normal[i]= facet->normal[i];
        }

        hull_faces[cntF]._id       = facet->id;
        hull_faces[cntF]._offset   = facet->offset;
        hull_faces[cntF]._toporient= (facet->toporient==True) ? true : false;
        cntF++;

        offset= facet->offset;

        if(contains_origin==false)
        {  continue;  }

        if(offset > 0)
        {  contains_origin= false; }
        else
        {
          offset= -offset;
          if(min_offset<=-1) { min_offset= offset; }
          else if(offset < min_offset)
          {  min_offset= offset;  } 
        }
     }
  }
  else
  {
    qh_freeqhull(!qh_ALL);  
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong)
    {
       fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",  totlong,  curlong); 
    }
    return 0;
  }
  if(contains_origin==false)
  {  _largest_ball_radius= 0;  }
  else
  {  _largest_ball_radius= min_offset;  }

  qh_freeqhull(!qh_ALL);  
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong)
  {
     fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",  totlong, curlong); 
  }
 
  _up_to_date= true;
  _vertex_list= qh vertex_list;
  _facet_list = qh facet_list;
  return 1;
}



//! Prints the content of the current convex hull.
//! \return 1 in case of success, 0 otherwise
int gpConvexHull::print()
{
  unsigned int i, j;

  printf("dimension= %d \n", _dimension); 

  printf("%d points \n", _points.size()); 

  if(_up_to_date)
  {
    printf("Convex hull is up to date.\n"); 
  }
  else
  {
    printf("Convex hull is NOT up to date.\n"); 
  }
  
  printf("convex hull: %d vertices %d simplicial faces\n", hull_vertices.size(), hull_faces.size()); 

  printf("input points: \n"); 
  for(i=0; i<_points.size(); i++)
  {
    printf("#%d: [", i); 
    for(j=0; j<_points[i].size(); j++)
    {
      printf(" %f", _points[i][j]);
    }
    printf(" ]\n"); 
  }

  if(!hull_faces.empty())
  {  
    printf("faces: \n"); 
    for(i=0; i<hull_faces.size(); i++)
    {
      printf(" face #%d\n", hull_faces[i].id());
      printf("    offset= %f\n", hull_faces[i].offset());
      printf("    normal: [");
      for(j=0; j<_dimension; j++)
      {
        printf("  %f ", hull_faces[i]._normal[j]);
      }
      printf(" ]\n");
      printf("    vertices: [");
      for(j=0; j<_dimension; j++)
      {
        printf("  %d ", hull_faces[i][j]);
      }
      printf(" ]\n");
    }
  }

  return 1;
}

//! Returns the radius of the largest ball centered on the origin and fully contained in the hull.
//! The function compute() must have been called before.
double gpConvexHull::largest_ball_radius()
{
  if(!_up_to_date)
  {
    printf("%s: %d: gpConvexHull::largest_ball_radius(): the conevx hull is not up to date; use the compute() function first.\n",__FILE__,__LINE__);
    return 0;
  }

  return _largest_ball_radius;
}


//! Initializes the input point set from the given p3d_vector3 array (with the same order so that, after hull computation,
//! indices in hull_vertices and hull_faces correspond to the indices in point_array).
gpConvexHull3D::gpConvexHull3D(p3d_vector3 *point_array, unsigned int nb_points)
{
  unsigned int i;

  _dimension= 3;
  _up_to_date= false;
  _largest_ball_radius= 0.0;
  _vertex_list= NULL;
  _facet_list = NULL;

  if(point_array==NULL)
  {
    printf("%s: %d: gpConvexHull3D::gpConvexHull3D(): point_array is NULL.\n",__FILE__,__LINE__);
    return;
  }
  if(nb_points < 4)
  {
    printf("%s: %d: gpConvexHull3D::gpConvexHull3D(): at least 4 points are needed to build a convex hull in 3D.\n",__FILE__,__LINE__);
    return;
  }

  _points.resize(nb_points);

  for(i=0; i<nb_points; i++)
  {
    _points[i].resize(3);
    _points[i][0]= point_array[i][0];
    _points[i][1]= point_array[i][1];
    _points[i][2]= point_array[i][2];
  }
}

//! Draws the current content of hull_vertices and hull_faces.
//! NB: The colors are set inside the function.
//! \return 1 in case of success, 0 otherwise
int gpConvexHull3D::draw()
{
  GLboolean enable_cullface, enable_lighting;
  unsigned int i, i1, i2, i3;
  GLint line_width, point_size;
  std::vector<double> center(3), normal;

  glGetBooleanv(GL_CULL_FACE, &enable_cullface);
  glGetBooleanv(GL_LIGHTING, &enable_lighting);
  glGetIntegerv(GL_LINE_WIDTH, &line_width);
  glGetIntegerv(GL_POINT_SIZE, &point_size);

  glDisable(GL_LIGHTING);

  glLineWidth(3);
  glColor3f(0, 1, 0);
  glBegin(GL_LINES);
   for(i=0; i<hull_faces.size(); i++)
   {
     i1= hull_faces[i][0];
     i2= hull_faces[i][1];
     i3= hull_faces[i][2];

     glVertex3f(_points[i1][0], _points[i1][1], _points[i1][2]);
     glVertex3f(_points[i2][0], _points[i2][1], _points[i2][2]);

     glVertex3f(_points[i1][0], _points[i1][1], _points[i1][2]);
     glVertex3f(_points[i3][0], _points[i3][1], _points[i3][2]);

     glVertex3f(_points[i2][0], _points[i2][1], _points[i2][2]);
     glVertex3f(_points[i3][0], _points[i3][1], _points[i3][2]);
   }
  glEnd();

  glBegin(GL_LINES);
   for(i=0; i<hull_faces.size(); i++)
   {
     i1= hull_faces[i][0];
     i2= hull_faces[i][1];
     i3= hull_faces[i][2];

     normal= hull_faces[i].normal();

     center[0]= ( _points[i1][0] + _points[i2][0] + _points[i3][0] )/3.0;
     center[1]= ( _points[i1][1] + _points[i2][1] + _points[i3][1] )/3.0;
     center[2]= ( _points[i1][2] + _points[i2][2] + _points[i3][2] )/3.0;

     glVertex3f(center[0], center[1], center[2]);
     glVertex3f(center[0] + normal[0], center[1] + normal[1], center[2] + normal[2]);
   }
  glEnd();


  glColor3f(1, 0, 0);
  glPointSize(6);
  glBegin(GL_POINTS);
   for(i=0; i<hull_vertices.size(); i++)
   {
     glVertex3f(_points[hull_vertices[i]][0], _points[hull_vertices[i]][1], _points[hull_vertices[i]][2]);
   }
  glEnd();

//   glColor3f(1, 0, 1);
//   glPointSize(6);
//   glBegin(GL_POINTS);
//    for(i=0; i<_points.size(); i++)
//    {
//      glVertex3f(_points[i][0], _points[i][1], _points[i][2]);
//    }
//   glEnd();


  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  g3d_set_color_mat(Blue, NULL);
  glBegin(GL_TRIANGLES);
   for(i=0; i<hull_faces.size(); i++)
   {
     i1= hull_faces[i][0];
     i2= hull_faces[i][1];
     i3= hull_faces[i][2];

     normal= hull_faces[i].normal();

     glNormal3f(normal[0], normal[1], normal[2]);
     if(hull_faces[i].toporient())
     {
       glVertex3f(_points[i1][0], _points[i1][1], _points[i1][2]);
       glVertex3f(_points[i3][0], _points[i3][1], _points[i3][2]);
       glVertex3f(_points[i2][0], _points[i2][1], _points[i2][2]);
     }
     else
     {
       glVertex3f(_points[i1][0], _points[i1][1], _points[i1][2]);
       glVertex3f(_points[i2][0], _points[i2][1], _points[i2][2]);
       glVertex3f(_points[i3][0], _points[i3][1], _points[i3][2]);
     }
   }
  glEnd();


  if(enable_cullface)
  {  glEnable(GL_CULL_FACE);  }
  else
  {  glDisable(GL_CULL_FACE);  }

  if(enable_lighting)
  {  glEnable(GL_LIGHTING);  }
  else
  {  glDisable(GL_LIGHTING);  }

  glLineWidth(line_width);
  glPointSize(point_size);

  return 1;
}


//! Draws the hull face of given index.
//! NB: The colors are set inside the function.
//! \param face_index the index of the face in the face array of the calling gpConvexHull3D
//! \return 1 in case of success, 0 otherwise
int gpConvexHull3D::drawFace(unsigned int face_index)
{
  if(face_index > hull_faces.size())
  {
    printf("%s: %d: gpConvexHull3D::drawFace(): input face index (%d) exceeds the hull's number of faces (%d).\n",__FILE__,__LINE__,face_index,hull_faces.size());
    return 0;
  }

  unsigned int i1, i2, i3;
  double d= 0.01;  
  std::vector<double> normal;

  i1= hull_faces[face_index][0];
  i2= hull_faces[face_index][1];
  i3= hull_faces[face_index][2];

  normal= hull_faces[face_index].normal();

  g3d_set_color_mat(Red, NULL);
  glBegin(GL_TRIANGLES);
    glVertex3f(_points[i1][0], _points[i1][1], _points[i1][2]);
    glVertex3f(_points[i2][0], _points[i2][1], _points[i2][2]);
    glVertex3f(_points[i3][0], _points[i3][1], _points[i3][2]);
  glEnd();

  glBegin(GL_TRIANGLES);
    glVertex3f(_points[i1][0] + d*normal[0], _points[i1][1] + d*normal[1], _points[i1][2] + d*normal[2]);
    glVertex3f(_points[i2][0] + d*normal[0], _points[i2][1] + d*normal[1], _points[i2][2] + d*normal[2]);
    glVertex3f(_points[i3][0] + d*normal[0], _points[i3][1] + d*normal[1], _points[i3][2] + d*normal[2]);
  glEnd();

  return 1;
}

//! Gets the coordinates of a the points of a convex hull's face (triangle).
//! \param face_index the index of the face in the face array of the calling gpConvexHull3D
//! \param p1 where to copy the coordinates of the first point
//! \param p2 where to copy the coordinates of the second point
//! \param p3 where to copy the coordinates of the third point
//! \return 1 in case of success, 0 otherwise
int gpConvexHull3D::getFacePoints(unsigned int face_index, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
  if(face_index > hull_faces.size())
  {
    printf("%s: %d: gpConvexHull3D::getFacePoints(): input face index (%d) exceeds the hull's number of faces (%d).\n",__FILE__,__LINE__,face_index,hull_faces.size());
    return 0;
  }

  p1[0]= _points[hull_faces[face_index][0]][0];
  p1[1]= _points[hull_faces[face_index][0]][1];
  p1[2]= _points[hull_faces[face_index][0]][2];

  p2[0]= _points[hull_faces[face_index][1]][0];
  p2[1]= _points[hull_faces[face_index][1]][1];
  p2[2]= _points[hull_faces[face_index][1]][2];

  p3[0]= _points[hull_faces[face_index][2]][0];
  p3[1]= _points[hull_faces[face_index][2]][1];
  p3[2]= _points[hull_faces[face_index][2]][2];

  return 1;
}


//! Initializes the input point set from the given array (with the same order so that, after hull computation,
//! indices in hull_vertices and hull_faces correspond to the indices in point_array).
gpConvexHull6D::gpConvexHull6D(double (*point_array)[6], unsigned int nb_points)
{
  unsigned int i, j;

  _dimension= 6;
  _up_to_date= false;
  _largest_ball_radius= 0.0;
  _vertex_list= NULL;
  _facet_list = NULL;

  if(point_array==NULL)
  {
    printf("%s: %d: gpConvexHull6D::gpConvexHull6D(): point_array is NULL.\n",__FILE__,__LINE__);
    return;
  }
  if(nb_points < 7)
  {
    printf("%s: %d: gpConvexHull6D::gpConvexHull6D(): at least 7 points are needed to build a convex hull in 6D.\n",__FILE__,__LINE__);
    return;
  }

  _points.resize(nb_points);
 
  for(i=0; i<nb_points; i++)
  {
    _points[i][j]= point_array[i][j];
  }

}

