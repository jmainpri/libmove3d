
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

//! Default gpRidge constructor (dimension= 2, size= 2)
gpRidge::gpRidge()
{
  _dimension= 2;
  _vertices.resize(_dimension);
  _id= 0;
  _toporient= true;
}

gpRidge::~gpRidge()
{
  _vertices.clear();
}

//! \param dimension desired dimension of the vertex space
//! \param vertex_number desired number of vertices of the ridge
gpRidge::gpRidge(unsigned int dimension, unsigned int vertex_number)
{
  if( dimension < 2)
  { 
    printf("%s: %d:  gpRidge::gpRidge(unsigned int): the space dimension must be > 1.\n",__FILE__,__LINE__);
    dimension= 2; 
  }
  if( vertex_number < dimension)
  { 
    printf("%s: %d:  gpRidge::gpRidge(unsigned int): the ridge's number of vertices must be >= %d.\n",__FILE__,__LINE__,dimension);
    vertex_number= dimension; 
  }

  _dimension= dimension;
  _vertices.resize(vertex_number);
  _id= 0;
  _toporient= true;
}

//! Resizes the ridge's number of vertices.
//! \warning private member. It should not be accessed from outside.
//! \param size the new number of vertices
//! \return 1 in case of success, 0 otherwise
int gpRidge::resize(unsigned int size)
{
  if(size < 2)
  {
    printf("%s: %d: gpRidge::resize(unsigned int): the number of vertices must be > 1.\n",__FILE__,__LINE__);
    return 0;
  }
  _vertices.resize(size);

  return 1;
}

//! Operator to access the vertex array of the ridge.
//! \param i index in the vertex array of the ridge (starts from 0)
//! \return the i-th element of the vertex index array of the ridge
unsigned int gpRidge::operator [] (const unsigned int i) const
{
  if(i > _vertices.size()-1)
  {
    printf("%s: %d: gpRidge::operator []: index (%d) exceeds the ridge's size (%d).\n",__FILE__,__LINE__,i,_vertices.size());
    return 0;
  }

  return _vertices[i];
}


//! Operator to access the vertex array of the ridge.
//! \param i index in the vertex array of the ridge (starts from 0)
//! \return a reference to the i-th element of the vertex index array of the ridge
unsigned int& gpRidge::operator [] (const unsigned int i)
{
  if(i > _vertices.size()-1)
  {
    printf("%s: %d: gpRidge::operator []: index (%d) exceeds the ridge's size (%d).\n",__FILE__,__LINE__,i,_vertices.size());
    return _vertices[0];
  }

  return _vertices[i];
}



//! Default gpFace constructor (dimension= 2, size= 2)
gpFace::gpFace()
{
  _dimension= 2;
  _vertices.resize(_dimension);
  _normal.resize(_dimension);
  _center.resize(_dimension);
  _id= 0;
  _offset= 0.0;
}

gpFace::~gpFace()
{
  _vertices.clear();
  _normal.clear();
  _center.clear();
  _ridges.clear();
}

//! \param dimension desired dimension of the vertex space
//! \param vertex_number desired number of vertices of the face
gpFace::gpFace(unsigned int dimension, unsigned int vertex_number)
{
  if( dimension < 2)
  { 
    printf("%s: %d:  gpFace::gpFace(unsigned int): the space dimension must be > 1.\n",__FILE__,__LINE__);
    dimension= 2; 
  }
  if( vertex_number < dimension)
  { 
    printf("%s: %d:  gpFace::gpFace(unsigned int): the face's number of vertices must be >= %d.\n",__FILE__,__LINE__,dimension);
    vertex_number= dimension; 
  }

  _dimension= dimension;
  _vertices.resize(vertex_number);
  _normal.resize(_dimension);
  _center.resize(_dimension);
  _id= 0;
  _offset= 0.0;
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return the i-th element of the vertex index array of the face
unsigned int gpFace::operator [] (const unsigned int i) const
{
  if(i > _vertices.size()-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's size (%d).\n",__FILE__,__LINE__,i,_vertices.size());
    return 0;
  }

  return _vertices[i];
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return a reference to the i-th element of the vertex index array of the face
unsigned int& gpFace::operator [] (const unsigned int i)
{
  if(i > _vertices.size()-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's size (%d).\n",__FILE__,__LINE__,i,_vertices.size());
    return _vertices[0];
  }

  return _vertices[i];
}


//! Changes the face's dimension (i.e. dimension of the space of the points).
//! \warning private member. It should not be accessed from outside.
//! \param dim the new dimension
//! \return 1 in case of success, 0 otherwise
int gpFace::setDimension(unsigned int dim)
{
  if(dim < 2)
  {
    printf("%s: %d: gpFace::setDimension(unsigned int): face dimension must be > 1.\n",__FILE__,__LINE__);
    return 0;
  }

  _dimension= dim;
  _normal.resize(_dimension);
  _center.resize(_dimension);

  return 1;
}

//! Resizes the face's number of vertices.
//! \warning private member. It should not be accessed from outside.
//! \param size the new number of vertices
//! \return 1 in case of success, 0 otherwise
int gpFace::resize(unsigned int size)
{
  if(size < 2)
  {
    printf("%s: %d: gpFace::resize(unsigned int): the number of vertices must be > 1.\n",__FILE__,__LINE__);
    return 0;
  }
  _vertices.resize(size);

  return 1;
}

//! Changes the ridge number of a face.
//! \param size the new ridge number
//! \return 1 in case of success, 0 otherwise
int gpFace::resizeRidgeNumber(unsigned int size)
{
  _ridges.resize(size);

  return 1;
}

//! Prints, in stdout, the content of a face.
//! \return 1 in case of success, 0 otherwise
int gpFace::print()
{
  unsigned int i, j;

  printf(" id= %d\n", _id);
  printf(" offset= %f\n", _offset);

  printf(" normal: [");
  for(i=0; i<_normal.size(); i++)
  {
    printf("  %f ", _normal[i]);
  }
  printf(" ]\n");

  printf(" center: [");
  for(i=0; i<_center.size(); i++)
  {
    printf("  %f ", _center[i]);
  }
  printf(" ]\n");

  printf("    %d vertices: [", _vertices.size());
  for(i=0; i<_vertices.size(); i++)
  {
    printf("  %d ", _vertices[i]);
  }
  printf(" ]\n");

  printf("    %d ridges: \n", _ridges.size());
  for(i=0; i<_ridges.size(); i++)
  {
    printf("              [");
    for(j=0; j<_ridges[i].nbVertices(); j++)
    {
      printf("    %d ", _ridges[i][j]);
    }
    printf("  ]\n");
  }

  return 1;
}


//! Reverse the order of the vertices in the array of vertices of the calling face.
//! \return 1 in case of success, 0 otherwise
int gpFace::reverseVertexOrder()
{
  unsigned int i, n, tmp;


  n= (unsigned int) (_vertices.size()/2.0);

  for(i=0; i<n; i++)
  {
    tmp= _vertices[_vertices.size() - 1 - i];
    _vertices[_vertices.size() - 1 - i]= _vertices[i];
    _vertices[i]= tmp;
  }

  return 1;
}


//! Orders the vertices of a  face according to its ridges.
//! This is needed for two reasons:
//! - Qhull returns the vertices in a face with an arbitrary order if it is non-simplicial
//! - if face merging is enabled, the toporient field of a facet is no longer valid and the ridge orientation
//!   must be used instead
//! This function is only used in 3D.
//! \return 1 in case of success, 0 otherwise
int gpFace::orderFromRidges()
{  
  unsigned int i, count, search;
  std::list<gpRidge> r;
  std::list<gpRidge>::iterator iter;
  std::vector<unsigned int> new_face;

  if(_dimension!=3)
  {
    printf("%s: %d: gpFace::orderFromRidges(): the point space dimension must be 3.\n",__FILE__,__LINE__);
    return 0;
  }

  count= 0;
  for(i=0; i<_ridges.size(); i++)
  {
    if(_ridges[i].nbVertices()!=2)
    {
      printf("%s: %d: gpFace::orderFromRidges(): the ridges must have exactly 2 vertices.\n",__FILE__,__LINE__);
      return 0;
    }
  }

  if(_vertices.size()!=_ridges.size())
  {
     printf("%s: %d: gpFace::orderFromRidges(): the face should have the same numbers of vertices and ridges.\n",__FILE__,__LINE__);
    return 0;
  }

  for(i=1; i<_ridges.size(); i++)
  {
    r.push_back(_ridges[i]);
  }

  new_face.reserve(_vertices.size());

  if(!_ridges[0].toporient())
  {
    new_face.push_back(_ridges[0][0]);
    new_face.push_back(_ridges[0][1]);
  }
  else
  {
    new_face.push_back(_ridges[0][1]);
    new_face.push_back(_ridges[0][0]);
  }


  while(!r.empty())
  {
    search= new_face.back(); 
    for(iter=r.begin(); iter!=r.end(); iter++)
    {  
       if( (*iter)[0]==search )
       {
         new_face.push_back((*iter)[1]);
         r.erase(iter);
         break;
       }
       if( (*iter)[1]==search )
       {
         new_face.push_back((*iter)[0]);
         r.erase(iter);
         break;
       }
    }
  }

  for(i=0; i<_vertices.size(); i++)
  { _vertices[i]= new_face[i];  }

  return 1;
}


//! Default constructor of the class gpConvexHull.
//! All is left void.
gpConvexHull::gpConvexHull()
{
 _dimension= 0;
 _up_to_date= false;
 _largest_ball_radius= 0.0;
}

gpConvexHull::~gpConvexHull()
{
  _points.clear();
  hull_vertices.clear();
  hull_faces.clear();
}

//! Computes the convex hull of the point set stored in the calling gpConvexHull variable.
//! The user can choose to have simplicial or non-simplicial output facets and to enable/disable
//! post-merging of nearly coplanar facets.
//! \param simplicial_facets selects wether the computed facets will be simplicial or not
//! \param postMergingCentrumRadius centrum radius for post-merging: merges facets when the centrum is less than the given value from a neighboring hyperplane. This allows Qhull to merge the hull facets that are adjacent and nearly coplanar.
//! Set postMergingCentrumRadius to a negative or null value to disable post-merging.
//! \param verbose selects if Qhull will print its messages in the console or not 
//! \return 1 in case of success, 0 otherwise.
//! NB: this function relies on Qhull library's functions and frees the memory used by Qhull once
//! the convex hull computation is done (TODO: check this last point (memory management)).
int gpConvexHull::compute(bool simplicial_facets, double postMergingCentrumRadius, bool verbose)
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
  unsigned int i, j, cntV= 0, cntF= 0, cntR= 0;
  double offset= 0, min_offset=-1.0;
  unsigned int numpoints;   // number of points
  coordT *point_array;      // array of coordinates for each point
  boolT ismalloc;           // True if qhull should free points in qh_freeqhull() or reallocation
  char flags[128]; // option flags for qhull, see qh_opt.htm
                               // OPTION 'Qt' is mandatory because we need the output facets to be simplicial
  FILE *outfile= stdout;    // output from qh_produce_output()			
	                    // use NULL to skip qh_produce_output() 
  outfile= NULL;
  FILE *errfile= stderr;    // error messages from qhull code

  int exitcode;             // 0 if no error from qhull
  pointT *centrum;
  vertexT *vertex, **vertexp; //vertex is used by the macro FORALLvertices, vertexp is used by the macro FOREACHvertex
  facetT *facet;  // used by the macro FORALLfacets
  ridgeT *ridge, **ridgep; // used by the macro FOREACHridge_
  int curlong, totlong;	  // memory remaining after qh_memfreeshort

  
  // used options:
  // s: print summary for the convex hull
  // -Q11: copy normals and recompute centrums for tricoplanar facets
  // -Qt: triangulated output
  // -C: facet post-merging
  if(simplicial_facets)
  {   
    if(postMergingCentrumRadius > 0)
    {  sprintf(flags, "qhull s -Q11 -Qt -C%f", postMergingCentrumRadius);  }
    else
    {  sprintf(flags, "qhull s -Q11 -Qt");  }
  }
  else
  {    
    if(postMergingCentrumRadius > 0)
    {  sprintf(flags, "qhull s -C%f", postMergingCentrumRadius);  }
    else
    {  sprintf(flags, "qhull s");  }
  }

  if(verbose)
  {
     outfile= stdout;
     errfile= stderr;
  }
  else
  {
     outfile= NULL;
     errfile= fopen("/dev/null", "w");
  }

  // initialize numpoints, point_array[], ismalloc here
  numpoints= _points.size();
  point_array= (coordT *) malloc(_dimension*numpoints*sizeof(coordT)); //use malloc to allocate memory because Qhull uses free()
  for(i=0; i<numpoints; i++)
  {
    if(_points[i].size()!=_dimension)
    {
      printf("%s: %d: gpConvexHull::compute(): an input point has an incorrect dimension (dimension is must be >= 2%d instead of %d).\n",__FILE__,__LINE__,_points[i].size(),_dimension); 
      if(!verbose) {  fclose(errfile);  }
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
        hull_faces[cntF].setDimension(_dimension);
        hull_faces[cntF].resize(qh_setsize(facet->vertices));

        cntV= 0;
        FOREACHvertex_(facet->vertices)
        {
          hull_faces[cntF][cntV]=  qh_pointid(vertex->point);
          cntV++;
        } 
        if(facet->toporient==True)
        {  hull_faces[cntF].reverseVertexOrder(); }
    
        centrum= qh_getcentrum(facet);
        for(i=0; i<_dimension; i++)
        {
          hull_faces[cntF]._normal[i]= facet->normal[i];
          hull_faces[cntF]._center[i]= centrum[i];
        }

        hull_faces[cntF]._id       = facet->id;
        hull_faces[cntF]._offset   = facet->offset;

        hull_faces[cntF].resizeRidgeNumber(qh_setsize(facet->ridges));
        cntR= 0;
        FOREACHridge_(facet->ridges)
        {
          hull_faces[cntF]._ridges[cntR].resize(qh_setsize(ridge->vertices));
          if(ridge->top == facet) { hull_faces[cntF]._ridges[cntR]._toporient= true; };
          if(ridge->bottom == facet) { hull_faces[cntF]._ridges[cntR]._toporient= false; };

          cntV= 0;
          FOREACHvertex_(ridge->vertices)
          {
            hull_faces[cntF]._ridges[cntR][cntV]= qh_pointid(vertex->point);
            cntV++;
          } 
          cntR++;
        }
        if( (_dimension==3) && (postMergingCentrumRadius > 0) )
        {
          hull_faces[cntF].orderFromRidges();
        }
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
//     printf("--------------------------------\n");
//     printf("----QHull reported an error.----\n");
//     printf("--------------------------------\n");
    if(!verbose) {  fclose(errfile);  }
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
     fprintf(errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",  totlong, curlong); 
  }
 
  _up_to_date= true;

  if(!verbose) {  fclose(errfile);  }

  return 1;
}

//! Gets the coordinates of the i-th input point used for convex hull computation.
//! \param i index of a point in the input point array (starts from 0)
//! \param coord where to copy the coordinates of the point
//! \return 1 in case of success, 0 otherwise
int gpConvexHull::pointCoordinates(unsigned int i, std::vector<double> &coord)
{
  if(i > _points.size()-1)
  {
    printf("%s: %d: gpConvexHull::pointCoordinates: index (%d) exceeds the hull's number of vertices (%d).\n",__FILE__,__LINE__,i,hull_vertices.size());
    return 0;
  }
  
  coord= _points[i];

  return 1;
}


//! Prints the content of the current convex hull.
//! \return 1 in case of success, 0 otherwise
int gpConvexHull::print()
{
  unsigned int i, j, count;

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

  count= 0;
  for(i=0; i<hull_faces.size(); i++)
  {
    if(hull_faces[i].nbVertices()!=_dimension)
    {
      count++;
    }
  }
  
  printf("convex hull: %d vertices\n", hull_vertices.size());
  printf("             %d faces (%d simplicial, %d non simplicial)\n", hull_faces.size(), hull_faces.size()-count,count);

//   printf("input points: \n"); 
//   for(i=0; i<_points.size(); i++)
//   {
//     printf("#%d: [", i); 
//     for(j=0; j<_points[i].size(); j++)
//     {
//       printf(" %f", _points[i][j]);
//     }
//     printf(" ]\n"); 
//   }

  if(!hull_faces.empty())
  {  
    printf("faces: \n"); 
    for(i=0; i<hull_faces.size(); i++)
    {
      printf(" face #%d\n", i);//hull_faces[i].id());
      //hull_faces[i].print();
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
    printf("%s: %d: gpConvexHull::largest_ball_radius(): the convex hull is not up to date; use the compute() function first.\n",__FILE__,__LINE__);
    return 0;
  }

  return _largest_ball_radius;
}


//! Tests if a given point is inside the convex hull.
//! \param point the point's coordinates
//! \param result true if the point is inside, false otherwise
//! \return 1 in case of success, 0 otherwise
int gpConvexHull::isPointInside(std::vector<double> point, bool &result)
{
  if(!_up_to_date)
  {
    printf("%s: %d: gpConvexHull::isPointInside(): the convex hull is not up to date; use the compute() function first.\n",__FILE__,__LINE__);
    return 0;
  }

  if(point.size()!=_dimension)
  {
    printf("%s: %d: gpConvexHull::isPointInside(): the input point does not have the same dimension than the hull's points.\n",__FILE__,__LINE__);
    return 0;
  }

  unsigned int i, j;
  double sum;

  for(i=0; i<hull_faces.size(); i++)
  {
    sum= 0.0;
    for(j=0; j<hull_faces[i]._normal.size(); j++)
    {
      sum+= hull_faces[i]._normal[j]*point[j];
    }
    sum+=  hull_faces[i]._offset;
    if(sum > 0)
    {
      result= false;
    }
  }

  result= true;
 
  return 1;
}


//! Initializes the input point set from the given p3d_vector3 array (with the same order so that, after hull computation,
//! indices in hull_vertices and hull_faces correspond to the indices in point_array).
gpConvexHull3D::gpConvexHull3D(p3d_vector3 *point_array, unsigned int nb_points)
{
  unsigned int i;

  _dimension= 3;
  _up_to_date= false;
  _largest_ball_radius= 0.0;

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
//! \param wireframe sets display mode (wireframe/solid)
//! \return 1 in case of success, 0 otherwise
int gpConvexHull3D::draw(bool wireframe)
{
  GLboolean enable_cullface, enable_lighting;
  unsigned int i, j, k;
  GLint line_width, point_size;
  std::vector<double> normal, center;

  glGetBooleanv(GL_CULL_FACE, &enable_cullface);
  glGetBooleanv(GL_LIGHTING, &enable_lighting);
  glGetIntegerv(GL_LINE_WIDTH, &line_width);
  glGetIntegerv(GL_POINT_SIZE, &point_size);

  glDisable(GL_LIGHTING);

  glLineWidth(3);
  glColor3f(0, 1, 0);

  for(i=0; i<hull_faces.size(); i++)
  {
    glBegin(GL_LINE_LOOP);
     for(j=0; j<hull_faces[i].nbVertices(); j++)
     {
       k= hull_faces[i][j];
       glVertex3f(_points[k][0], _points[k][1], _points[k][2]);
     }
     glEnd();
  }

  glColor3f(1, 0, 1);
  glBegin(GL_LINES);
   for(i=0; i<hull_faces.size(); i++)
   {
     normal= hull_faces[i].normal();
     center= hull_faces[i].center();

     glVertex3f(center[0], center[1], center[2]);
     glVertex3f(center[0] + normal[0], center[1] + normal[1], center[2] + normal[2]);
   }
  glEnd();

//   glColor3f(1, 0, 1);
//   glBegin(GL_LINES);
//    for(i=0; i<hull_faces.size(); i++)
//    {
//      normal= hull_faces[i].normal();
//      for(j=0; j<hull_faces[i].nbVertices(); j++)
//      {
//        k= hull_faces[i][j];
//        glVertex3f(_points[k][0], _points[k][1], _points[k][2]);
//        glVertex3f(_points[k][0] + normal[0], _points[k][1] + normal[1], _points[k][2] + normal[2]);
//      }
//    }
//   glEnd();


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

  if(!wireframe)
  {
      g3d_set_color_mat(Blue, NULL);
      for(i=0; i<hull_faces.size(); i++)
      {
        glBegin(GL_POLYGON);
          normal= hull_faces[i].normal();
          glNormal3f(normal[0], normal[1], normal[2]);
          for(j=0; j<hull_faces[i].nbVertices(); j++)
          {
            k= hull_faces[i][j];
            glVertex3f(_points[k][0], _points[k][1], _points[k][2]);
          }
        glEnd();
      }
  }

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

  unsigned int i, k;
  double d= 0.01;
  std::vector<double> normal, center;

  normal= hull_faces[face_index].normal();
  center= hull_faces[face_index].center();

  g3d_set_color_mat(Red, NULL);

  glBegin(GL_POLYGON);
    glNormal3f(normal[0], normal[1], normal[2]);
    for(i=0; i<hull_faces[face_index].nbVertices(); i++)
    {
      k= hull_faces[face_index][i];
// printf("df: index %d\n", k);
// printf("%f %f %f \n",_points[k][0], _points[k][1], _points[k][2]);
      glVertex3f(_points[k][0], _points[k][1], _points[k][2]);
    }
  glEnd();

  glBegin(GL_LINES);
    glVertex3f(center[0], center[1], center[2]);
    glVertex3f(center[0] + d*normal[0], center[1] + d*normal[1], center[2] + d*normal[2]);
  glEnd();

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
    _points[i].resize(6);
    _points[i][0]= point_array[i][0];
    _points[i][1]= point_array[i][1];
    _points[i][2]= point_array[i][2];
    _points[i][3]= point_array[i][3];
    _points[i][4]= point_array[i][4];
    _points[i][5]= point_array[i][5];
  }

}

