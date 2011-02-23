


#include <stdio.h>
#include <vector>
#include <algorithm>

#ifdef USE_SHADERS
  #include <GL/glew.h>
#else
 #include <GL/glu.h>
#endif

#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/include/gpConvexHull.h"

////! see qhull_interface.cpp for more information
// extern "C"
// {
//   #include "qhull/qhull_a.h"
// }
#include "../graspPlanning/qhull/qhull_a.h"

#include "../include/p3d_matrix.h"

//! Default gpRidge constructor (dimension= 2, size= 2)
gpRidge::gpRidge()
{
  dimension_= 2;
  vertices_.resize(dimension_);
  id_= 0;
  toporient_= true;
}

gpRidge::~gpRidge()
{
  vertices_.clear();
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

  dimension_= dimension;
  vertices_.resize(vertex_number);
  id_= 0;
  toporient_= true;
}

//! Resizes the ridge's number of vertices.
//! \warning private member. It should not be accessed from outside.
//! \param size the new number of vertices
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpRidge::resize(unsigned int size)
{
  if(size < 2)
  {
    printf("%s: %d: gpRidge::resize(unsigned int): the number of vertices must be > 1.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  vertices_.resize(size);

  return GP_OK;
}

//! Operator to access the vertex array of the ridge.
//! \param i index in the vertex array of the ridge (starts from 0)
//! \return the i-th element of the vertex index array of the ridge
unsigned int gpRidge::operator [] (const unsigned int i) const
{
  if(i > vertices_.size()-1)
  {
    printf("%s: %d: gpRidge::operator []: index (%d) exceeds the ridge's size (%d).\n",__FILE__,__LINE__,i,vertices_.size());
    return GP_ERROR;
  }

  return vertices_[i];
}


//! Operator to access the vertex array of the ridge.
//! \param i index in the vertex array of the ridge (starts from 0)
//! \return a reference to the i-th element of the vertex index array of the ridge
unsigned int& gpRidge::operator [] (const unsigned int i)
{
  if(i > vertices_.size()-1)
  {
    printf("%s: %d: gpRidge::operator []: index (%d) exceeds the ridge's size (%d).\n",__FILE__,__LINE__,i,vertices_.size());
    return vertices_[0];
  }

  return vertices_[i];
}



//! Default gpFace constructor (dimension= 2, size= 2)
gpFace::gpFace()
{
  dimension_= 2;
  vertices_.resize(dimension_);
  normal_.resize(dimension_);
  center_.resize(dimension_);
  id_= 0;
  offset_= 0.0;
}

gpFace::~gpFace()
{
  vertices_.clear();
  normal_.clear();
  center_.clear();
  ridges_.clear();
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

  dimension_= dimension;
  vertices_.resize(vertex_number);
  normal_.resize(dimension_);
  center_.resize(dimension_);
  id_= 0;
  offset_= 0.0;
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return the i-th element of the vertex index array of the face
unsigned int gpFace::operator [] (const unsigned int i) const
{
  if(i > vertices_.size()-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's size (%d).\n",__FILE__,__LINE__,i,vertices_.size());
    return 0;
  }

  return vertices_[i];
}


//! Operator to access the vertex array of the face.
//! \param i index in the vertex array of the face (starts from 0)
//! \return a reference to the i-th element of the vertex index array of the face
unsigned int& gpFace::operator [] (const unsigned int i)
{
  if(i > vertices_.size()-1)
  {
    printf("%s: %d: gpFace::operator []: index (%d) exceeds the face's size (%d).\n",__FILE__,__LINE__,i,vertices_.size());
    return vertices_[0];
  }

  return vertices_[i];
}


//! Changes the face's dimension (i.e. dimension of the space of the points).
//! \warning private member. It should not be accessed from outside.
//! \param dim the new dimension
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::setDimension(unsigned int dim)
{
  if(dim < 2)
  {
    printf("%s: %d: gpFace::setDimension(unsigned int): face dimension must be > 1.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  dimension_= dim;
  normal_.resize(dimension_);
  center_.resize(dimension_);

  return GP_OK;
}

//! Resizes the face's number of vertices.
//! \warning private member. It should not be accessed from outside.
//! \param size the new number of vertices
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::resize(unsigned int size)
{
  if(size < 2)
  {
    printf("%s: %d: gpFace::resize(unsigned int): the number of vertices must be > 1.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  vertices_.resize(size);

  return GP_OK;
}

//! Changes the ridge number of a face.
//! \param size the new ridge number
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::resizeRidgeNumber(unsigned int size)
{
  ridges_.resize(size);

  return GP_OK;
}

//! Prints, in stdout, the content of a face.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::print()
{
  unsigned int i, j;

  printf(" id= %d\n", id_);
  printf(" offset= %f\n", offset_);

  printf(" normal: [");
  for(i=0; i<normal_.size(); i++)
  {
    printf("  %f ", normal_[i]);
  }
  printf(" ]\n");

  printf(" center: [");
  for(i=0; i<center_.size(); i++)
  {
    printf("  %f ", center_[i]);
  }
  printf(" ]\n");

  printf("    %d vertices: [", vertices_.size());
  for(i=0; i<vertices_.size(); i++)
  {
    printf("  %d ", vertices_[i]);
  }
  printf(" ]\n");

  printf("    %d ridges: \n", ridges_.size());
  for(i=0; i<ridges_.size(); i++)
  {
    printf("              [");
    for(j=0; j<ridges_[i].nbVertices(); j++)
    {
      printf("    %d ", ridges_[i][j]);
    }
    printf("  ]\n");
  }

  return GP_OK;
}


//! Reverse the order of the vertices in the array of vertices of the calling face.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::reverseVertexOrder()
{
  unsigned int i, n, tmp;


  n= (unsigned int) (vertices_.size()/2.0);

  for(i=0; i<n; i++)
  {
    tmp= vertices_[vertices_.size() - 1 - i];
    vertices_[vertices_.size() - 1 - i]= vertices_[i];
    vertices_[i]= tmp;
  }

  return GP_OK;
}


//! Orders the vertices of a  face according to its ridges.
//! This is needed for two reasons:
//! - Qhull returns the vertices in a face with an arbitrary order if it is non-simplicial
//! - if face merging is enabled, the toporient field of a facet is no longer valid and the ridge orientation
//!   must be used instead
//! This function is only used in 3D.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFace::orderFromRidges()
{  
  unsigned int i, count, search;
  std::list<gpRidge> r;
  std::list<gpRidge>::iterator iter;
  std::vector<unsigned int> new_face;

  if(dimension_!=3)
  {
    printf("%s: %d: gpFace::orderFromRidges(): the point space dimension must be 3.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  count= 0;
  for(i=0; i<ridges_.size(); i++)
  {
    if(ridges_[i].nbVertices()!=2)
    {
      printf("%s: %d: gpFace::orderFromRidges(): the ridges must have exactly 2 vertices.\n",__FILE__,__LINE__);
      return GP_ERROR;
    }
  }

  if(vertices_.size()!=ridges_.size())
  {
     printf("%s: %d: gpFace::orderFromRidges(): the face should have the same numbers of vertices and ridges.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  for(i=1; i<ridges_.size(); i++)
  {
    r.push_back(ridges_[i]);
  }

  new_face.reserve(vertices_.size());

  if(!ridges_[0].toporient())
  {
    new_face.push_back(ridges_[0][0]);
    new_face.push_back(ridges_[0][1]);
  }
  else
  {
    new_face.push_back(ridges_[0][1]);
    new_face.push_back(ridges_[0][0]);
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

  for(i=0; i<vertices_.size(); i++)
  { vertices_[i]= new_face[i];  }

  return GP_OK;
}


//! Default constructor of the class gpConvexHull.
//! All is left void.
gpConvexHull::gpConvexHull()
{
 dimension_= 0;
 up_to_date_= false;
 largest_ball_radius_= 0.0;
}

gpConvexHull::gpConvexHull(const std::vector< std::vector<double> > &points)
{
  int result;

  reset();

  result= setPoints(points);

  if(result==GP_ERROR)
  {
    printf("%s: %d: gpConvexHull::gpConvexHull(): there is something wrong with the input set of points.\n",__FILE__,__LINE__);
    return;
  }
}

gpConvexHull::~gpConvexHull()
{
  points_.clear();
  hull_vertices.clear();
  hull_faces.clear();
}


//! Sets the point set that will then be used when calling compute() or voronoi().
//! \param points a vector of vectors whose size will give the dimension of the space
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull::setPoints(const std::vector< std::vector<double> > &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull::setPoints(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j;

  reset();

  if(points.empty())
  {
    printf("%s: %d: gpConvexHull::setPoints(): input point set is empty.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  dimension_= points[0].size();

  if(points.size() < dimension_ + 1)
  {
    printf("%s: %d: gpConvexHull::setPoints(): at least %d points are needed to build a convex hull in %dD.\n",__FILE__,__LINE__,dimension_+1,dimension_);
    return GP_ERROR;
  }

  points_.resize( points.size() );

  for(i=0; i<points.size(); ++i)
  {
    points_[i].resize(dimension_);
    if(points[i].size()!=dimension_)
    {
      printf("%s: %d: gpConvexHull::setPoints(): all the input points should be of dimension %d.\n",__FILE__,__LINE__,dimension_);
      return GP_ERROR;
    } 
    else
    {  
       for(j=0; j<dimension_; ++j)
       {
         points_[i].at(j)= points[i].at(j); 
       }
    }
  }

  return GP_OK;
}

//! Reinitializes everything.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull::reset()
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull::reset(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  dimension_          = 0;
  up_to_date_         = false;
  largest_ball_radius_= 0.0;

  points_.clear();
  hull_vertices.clear();
  hull_faces.clear();

  return GP_OK;
}

//! Computes the convex hull of the point set stored in the calling gpConvexHull variable.
//! The user can choose to have simplicial or non-simplicial output facets and to enable/disable
//! post-merging of nearly coplanar facets.
//! \param simplicial_facets selects wether the computed facets will be simplicial or not
//! \param postMergingCentrumRadius centrum radius for post-merging: merges facets when the centrum is less than the given value from a neighboring hyperplane. This allows Qhull to merge the hull facets that are adjacent and nearly coplanar.
//! Set postMergingCentrumRadius to a negative or null value to disable post-merging.
//! \param verbose selects if Qhull will print its messages in the console or not 
//! \return GP_OK in case of success, GP_ERROR otherwise.
//! NB: this function relies on Qhull library's functions and frees the memory used by Qhull once
//! the convex hull computation is done (TODO: check this last point (memory management)).
int gpConvexHull::compute(bool simplicial_facets, double postMergingCentrumRadius, bool verbose)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull::compute(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(points_.empty())
  {
    printf("%s: %d: gpConvexHull::compute(): no input points have been set.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(dimension_ < 2)
  {
    printf("%s: %d: gpConvexHull::compute(): points dimension must be >= 2.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(points_.size() < dimension_+1)
  {
    printf("%s: %d: gpConvexHull::compute(): there must be at least %d input points to compute the convex hull (dimension= %d).\n",__FILE__,__LINE__,dimension_+1, dimension_);
    return GP_ERROR;
  }
  
  bool contains_origin= false;
  unsigned int i, j, cntV= 0, cntF= 0, cntR= 0;
  double offset= 0, minoffset_=-1.0;
  unsigned int numpoints;   // number of points
  coordT *point_array= NULL;      // array of coordinates for each point
  boolT ismalloc;           // True if qhull should free points in qh_freeqhull() or reallocation
  char flags[64]; // option flags for qhull, see qh_opt.htm
                               // OPTION 'Qt' is mandatory because we need the output facets to be simplicial
  FILE *outfile= stdout;    // output from qh_produce_output()			
	                    // use NULL to skip qh_produce_output() 
  outfile= NULL;
  FILE *errfile= stderr;    // error messages from qhull code

  int exitcode;             // 0 if no error from qhull
  pointT *centrum= NULL;
  vertexT *vertex= NULL, **vertexp= NULL; //vertex is used by the macro FORALLvertices, vertexp is used by the macro FOREACHvertex
  facetT *facet= NULL;  // used by the macro FORALLfacets
  ridgeT *ridge= NULL, **ridgep= NULL; // used by the macro FOREACHridge_
  int curlong, totlong;	  // memory remaining after qh_memfreeshort

  
  // used options:
  // s: print summary for the convex hull
  // -Q11: copy normals and recompute centrums for tricoplanar facets
  // -Qt: triangulated output
  // -C: facet post-merging
  if(simplicial_facets)
  {   
    if(postMergingCentrumRadius > 0.0)
    {  sprintf(flags, "qhull s -Q11 -Qt -C%f", postMergingCentrumRadius);  }
    else
    {  sprintf(flags, "qhull s -Q11 -Qt");  }
  }
  else
  {    
    if(postMergingCentrumRadius > 0.0)
    {  sprintf(flags, "qhull s Tc -C%f", postMergingCentrumRadius);  }
    else
    {  sprintf(flags, "qhull s");  }
  }

  if(verbose)
  {
     printf("QHULL OPTIONS= %s\n", flags);
     outfile= stdout;
     errfile= stdout;
  }
  else
  {
     outfile= NULL;
     errfile= fopen("/dev/null", "w");
  }

  // initialize numpoints, point_array[], ismalloc here
  numpoints= points_.size();
  point_array= (coordT *) malloc(dimension_*numpoints*sizeof(coordT)); //use malloc to allocate memory because Qhull uses free()

  if(point_array==NULL)
  {
    printf("%s: %d: gpConvexHull::compute(): failed to allocate %d coords.\n",__FILE__,__LINE__,dimension_*numpoints);
    return GP_ERROR;
  }

  for(i=0; i<numpoints; ++i)
  {
    if(points_[i].size()!=dimension_)
    {
      printf("%s: %d: ConvexHull::compute(): an input point has an incorrect dimension (dimension is %d instead of %d).\n",__FILE__,__LINE__,points_[i].size(),dimension_); 
      if(!verbose) {  fclose(errfile);  }
      return GP_ERROR;
    }
    for(j=0; j<dimension_; ++j)
    {
      point_array[dimension_*i+j]= points_[i][j]; 
    }
  }
  ismalloc= True;

  qh_freeqhull(!qh_ALL);
  qh_memfreeshort(&curlong, &totlong);

  exitcode= qh_new_qhull(dimension_, numpoints, point_array, ismalloc, flags, outfile, errfile);

  contains_origin= true;
  minoffset_= -1.0;

  if(!exitcode)  // if no error 
  {
     // get the hull vertices:
     hull_vertices.resize(qh_qh.num_vertices);
     cntV= 0;

     // strangely, the following (getting hull vertices and facets) is not working anymore
     // with the old version of (qhull-2003.1-14, with .so library) but works
     // with qhull-2010.1. Since the embedding of qhull-2010.1 code in BioMove3D, it is OK.
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
        hull_faces[cntF].setDimension(dimension_);
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
        for(i=0; i<dimension_; i++)
        {
          hull_faces[cntF].normal_[i]= facet->normal[i];
          hull_faces[cntF].center_[i]= centrum[i];
        }

        hull_faces[cntF].id_       = facet->id;
        hull_faces[cntF].offset_   = facet->offset;

        hull_faces[cntF].resizeRidgeNumber(qh_setsize(facet->ridges));
        cntR= 0;
        FOREACHridge_(facet->ridges)
        {
          hull_faces[cntF].ridges_[cntR].resize(qh_setsize(ridge->vertices));
          if(ridge->top == facet) { hull_faces[cntF].ridges_[cntR].toporient_= true; };
          if(ridge->bottom == facet) { hull_faces[cntF].ridges_[cntR].toporient_= false; };

          cntV= 0;
          FOREACHvertex_(ridge->vertices)
          {
            hull_faces[cntF].ridges_[cntR][cntV]= qh_pointid(vertex->point);
            cntV++;
          } 
          cntR++;
        }
        if( (dimension_==3) && (postMergingCentrumRadius > 0) )
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
          if(minoffset_<=-1) { minoffset_= offset; }
          else if(offset < minoffset_)
          {  minoffset_= offset;  } 
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
    return GP_ERROR;
  }
  if(contains_origin==false)
  {  largest_ball_radius_= 0;  }
  else
  {  largest_ball_radius_= minoffset_;  }

  qh_freeqhull(!qh_ALL);  
  qh_memfreeshort(&curlong, &totlong);
  if (curlong || totlong)
  {
     fprintf(errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",  totlong, curlong); 
  }
 
  up_to_date_= true;

  if(!verbose) {  fclose(errfile);  }

  return GP_OK;
}



//! Computes the voronoi regions of the point set stored in the calling gpConvexHull variable.
//! The function does not read into Qhulll's internal data but redirects Qhull's output into
//! a temporary file and read into this file.
//! \param verbose selects if Qhull will print its messages in the console or not 
//! \return GP_OK in case of success, GP_ERROR otherwise.
//! NB: this function relies on Qhull library's functions and frees the memory used by Qhull once
//! the computation is done (TODO: check this last point (memory management)).
int gpConvexHull::voronoi(bool verbose)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull::voronoi(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  bool result, atInfinity;
  unsigned int i, j, k;
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
  int curlong, totlong;	  // memory remaining after qh_memfreeshort

  char name[]= "/tmp/fileXXXXXX";
  int fd;
  std::vector<double> v;
  char buffer[256];
  std::istringstream iss;
  unsigned int index, site1, site2, dimension, nbVoronoiVertices, nbVoronoiRidges, nbElements;
  unsigned long int nbHypercubeVertices, mask;
  int bit;
  double coord, scaleFactor;
  std::vector< std::pair<double,double> > boundaries; // point set bounding box
  gpVoronoiRidge voronoiRidge;
  gpVoronoiCell cell;

  // used options:
  // they are set to get the voronoi vertices plus the voronoi ridges
  sprintf(flags, "qhull v -p -Fv -Qbb -Qx -Q11 -Qt");

  if(verbose)
  {  errfile= stderr;  }
  else
  {  errfile= fopen("/dev/null", "w");  }

  // we will write the output of qhull into a temporary file:
  fd= mkstemp(name);
  outfile= fdopen( fd, "w+");

  // We will use additional points to enclose the original point set in order to replace unbounded regions with bounded ones:
  nbHypercubeVertices= 1;
  nbHypercubeVertices <<= dimension_;
  numpoints= points_.size() + nbHypercubeVertices;

  point_array= (coordT *) malloc(dimension_*numpoints*sizeof(coordT)); //use malloc to allocate memory because Qhull uses free()

  boundaries.resize(dimension_);
  for(i=0; i<points_.size(); ++i)
  {
    if(points_[i].size()!=dimension_)
    {
      printf("%s: %d: gpConvexHull::voronoi(): an input point has an incorrect dimension (dimension is %d instead of %d).\n",__FILE__,__LINE__,points_[i].size(),dimension_); 
      if(!verbose) {  fclose(errfile);  }
      return GP_ERROR;
    }

    for(j=0; j<dimension_; ++j)
    {
      point_array[dimension_*i + j]= points_[i][j];

      if( (i==0) || (points_[i][j] < boundaries[j].first) )
      {
        boundaries[j].first= points_[i][j];
      }
      if( (i==0) || (points_[i][j] > boundaries[j].second) )
      {
        boundaries[j].second= points_[i][j];
      }
    }

  }

  // add the enclosing points (vertices of a hypercube)
  
  //scale up a little the bounding cube;
  scaleFactor= 1.1;
  for(i=0; i<dimension_; ++i)
  {
    if(boundaries[i].first < 0)
    {  boundaries[i].first*= scaleFactor; }
    else
    {  boundaries[i].first*= (1-scaleFactor); }

    if(boundaries[i].second > 0)
    {  boundaries[i].second*= scaleFactor; }
    else
    {  boundaries[i].second*= (1-scaleFactor); }
  }

  // fill the hypercube vertices:
  for(i=0; i<nbHypercubeVertices; ++i)
  {
    for(j=0; j<dimension_; ++j)
    {
      mask= 1;
      mask <<= j;
      bit= i & mask;
      if(bit==0)
      {
        point_array[dimension_*(points_.size()+i) + j]= boundaries[j].first;
      }
      else
      {
        point_array[dimension_*(points_.size()+i) + j]= boundaries[j].second;
      }
    }
  }


  ismalloc= True;

  exitcode= qh_new_qhull(dimension_, numpoints, point_array, ismalloc, flags, outfile, errfile);

  voronoi_vertices_.clear();
  voronoi_ridges_.clear();
  voronoi_cells_.clear();

  if(!exitcode)  // if no error 
  {
     qh_setvoronoi_all();

     // read the output file of qhull: 
     rewind(outfile);
     fgets(buffer, 256, outfile);
     iss.str(buffer);
     result= (iss >> dimension );
     if( !result || iss.fail() )
     {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 

     fgets(buffer, 256, outfile);
     iss.str(buffer);
     result= (iss >> nbVoronoiVertices );
     if( !result || iss.fail() )
     {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 

     v.resize(dimension);
     for(i=0; i<nbVoronoiVertices; ++i)
     {
       fgets(buffer, 256, outfile);
       iss.str(buffer);

       for(j=0; j<dimension; ++j)
       {
         result= (iss >> coord );
         if( !result || iss.fail() )
         {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 
         v.at(j)= coord;
       }
       voronoi_vertices_.push_back(v);
     }

     fgets(buffer, 256, outfile);
     iss.str(buffer);
     result= (iss >> nbVoronoiRidges );
     if( !result || iss.fail() )
     {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 

     v.resize(dimension);
     for(i=0; i<nbVoronoiRidges; ++i)
     {
       atInfinity= false;

       fgets(buffer, 256, outfile);
       iss.str(buffer);
       result= (iss >> nbElements >> site1 >> site2 );
       if( !result || iss.fail() )
       {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 
       voronoiRidge.site1_id_= site1;
       voronoiRidge.site2_id_= site2;
       if(voronoiRidge.site1_id_ >= points_.size()) { voronoiRidge.site1_id_= UINT_MAX;  }
       if(voronoiRidge.site2_id_ >= points_.size()) { voronoiRidge.site2_id_= UINT_MAX;  }

       if(voronoiRidge.site1_id_==UINT_MAX && voronoiRidge.site2_id_==UINT_MAX )
       {  continue;  }

       voronoiRidge.vertices_.clear();
       for(j=0; j<nbElements-2; ++j)
       {
         result= (iss >> index );
         if( !result || iss.fail() )
         {   printf("%s: %d: error\n",__FILE__,__LINE__);   } 
       
         if(index==0) 
         {  atInfinity= true;  }
         else
         {  voronoiRidge.vertices_.push_back(index-1);  }
       }
       if(!atInfinity)
       {  voronoi_ridges_.push_back(voronoiRidge);  }
     }

     //////////////////////////////////////////////////////////////////
     // print the result:
//      printf("nbVVertices= %d, nbVRidges= %d\n", voronoi_vertices_.size(), voronoi_ridges_.size());
// 
//      for(i=0; i<voronoi_vertices_.size(); ++i)
//      {
//        printf("VVertex[%d]: ",i);
//        for(j=0; j<voronoi_vertices_[i].size(); ++j)
//        {
//          printf(" %f ", voronoi_vertices_[i][j]);
//        }
//        printf("\n");
//      }
// 
//      for(i=0; i<voronoi_ridges_.size(); ++i)
//      {
//        printf("VRidge[%d]: sites [%d %d] ",i,voronoi_ridges_[i].site1_id_,voronoi_ridges_[i].site2_id_);
//        for(j=0; j<voronoi_ridges_[i].vertices_.size(); ++j)
//        {
//          printf(" %d ", voronoi_ridges_[i].vertices_[j]);
//        }
//        printf("\n");
//      }
     //////////////////////////////////////////////////////////////////

     // compute the center of each ridge:
     for(i=0; i<voronoi_ridges_.size(); ++i)
     {
        voronoi_ridges_.at(i).center_.assign(dimension_, 0.0);

        for(j=0; j<voronoi_ridges_.at(i).vertices_.size(); ++j)
        {
          for(k=0; k<voronoi_ridges_.at(i).center_.size(); ++k)
          {
            voronoi_ridges_.at(i).center_.at(k)+=  voronoi_vertices_.at( voronoi_ridges_.at(i).vertices_.at(j) ).at(k);
          }
        }
        for(k=0; k<voronoi_ridges_.at(i).center_.size(); ++k)
        {  voronoi_ridges_.at(i).center_.at(k)/=  (double) voronoi_ridges_.at(i).vertices_.size();  }
     }

     // create the voronoi cells from the ridges:
     voronoi_cells_.resize(points_.size());
     for(i=0; i<points_.size(); ++i)
     {
       voronoi_cells_.at(i).site_id_= i;
       for(j=0; j<voronoi_ridges_.size(); ++j)
       {
         if( voronoi_ridges_.at(j).site1_id_==i || voronoi_ridges_.at(j).site2_id_==i )
         {
           voronoi_cells_.at(i).ridges_.push_back(j);
         }
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
    return GP_ERROR;
  }


  qh_freeqhull(!qh_ALL);  
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong)
  {
     fprintf(errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",  totlong, curlong); 
  }
 
  up_to_date_= true;

  if(!verbose)
  {  fclose(errfile);  }

  return GP_OK;
}

//! Gets the coordinates of the i-th input point used for convex hull computation.
//! \param i index of a point in the input point array (starts from 0)
//! \param coord where to copy the coordinates of the point
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull::pointCoordinates(unsigned int i, std::vector<double> &coord)
{
  if(i > points_.size()-1)
  {
    printf("%s: %d: gpConvexHull::pointCoordinates: index (%d) exceeds the hull's number of vertices (%d).\n",__FILE__,__LINE__,i,hull_vertices.size());
    return GP_ERROR;
  }
  
  coord= points_[i];

  return GP_OK;
}


//! Prints the content of the current convex hull.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull::print()
{
  unsigned int i, count;

  printf("dimension= %d \n", dimension_); 

  printf("%d points \n", points_.size()); 

  if(up_to_date_)
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
    if(hull_faces[i].nbVertices()!=dimension_)
    {
      count++;
    }
  }
  
  printf("convex hull: %d vertices\n", hull_vertices.size());
  printf("             %d faces (%d simplicial, %d non simplicial)\n", hull_faces.size(), hull_faces.size()-count,count);

//   printf("input points: \n"); 
//   for(i=0; i<points_.size(); i++)
//   {
//     printf("#%d: [", i); 
//     for(j=0; j<points_[i].size(); j++)
//     {
//       printf(" %f", points_[i][j]);
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

  return GP_OK;
}

//! Returns the radius of the largest ball centered on the origin and fully contained in the hull.
//! The function compute() must have been called before.
double gpConvexHull::largest_ball_radius()
{
  if(!up_to_date_)
  {
    printf("%s: %d: gpConvexHull::largest_ball_radius(): the convex hull is not up to date; use the compute() function first.\n",__FILE__,__LINE__);
    return 0;
  }

  return largest_ball_radius_;
}


//! Tests if a given point is inside the convex hull.
//! If it is the case, the function also gives the distance from the points
//! to the hull.
//! \param point the point's coordinates
//! \param result true if the point is inside, false otherwise
//! \param distance true if the point is inside, false otherwise
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull::isPointInside(std::vector<double> point, bool &inside, double &distance)
{
  if(!up_to_date_)
  {
    printf("%s: %d: gpConvexHull::isPointInside(): the convex hull is not up to date; use the compute() function first.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(point.size()!=dimension_)
  {
    printf("%s: %d: gpConvexHull::isPointInside(): the input point does not have the same dimension than the hull's points.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j;
  double sum, dmin= -1;

  for(i=0; i<hull_faces.size(); i++)
  {
    sum= 0.0;
    for(j=0; j<hull_faces[i].normal_.size(); j++)
    {
      sum+= hull_faces[i].normal_[j]*point[j];
    }
    sum+=  hull_faces[i].offset_;

    if(sum > 0)
    {
      inside= false;
      distance= -1;
      return GP_OK;
    }

    if(dmin==-1)
    {  dmin= -sum;  }
    else
    {
      if(-sum < dmin)
      {  dmin= -sum;  } 
    }
  }

  inside= true;
  distance= dmin;

  return GP_OK;
}

gpConvexHull3D::gpConvexHull3D(): gpConvexHull()
{
  dimension_= 3;
}

//! Initializes the input point set from the given p3d_vector3 array (with the same order so that, after hull computation,
//! indices in hull_vertices and hull_faces correspond to the indices in point_array).
gpConvexHull3D::gpConvexHull3D(p3d_vector3 *point_array, unsigned int nb_points)
{
  int result;

  dimension_= 3;
  up_to_date_= false;
  largest_ball_radius_= 0.0;

  result= setPoints(point_array, nb_points);

  if(result==GP_ERROR)
  {
    printf("%s: %d: gpConvexHull3D::gpConvexHull3D(): there is something wrong with the input set of points.\n",__FILE__,__LINE__);
    return;
  }
}


int gpConvexHull3D::setPoints(p3d_vector3 *point_array, unsigned int nb_points)
{
  unsigned int i;

  reset();

  if(point_array==NULL)
  {
    printf("%s: %d: gpConvexHull3D::gpConvexHull3D(): point_array is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(nb_points < 4)
  {
    printf("%s: %d: gpConvexHull3D::gpConvexHull3D(): at least 4 points are needed to build a convex hull in 3D.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  dimension_= 3;
  points_.resize(nb_points);

  for(i=0; i<nb_points; i++)
  {
    points_[i].resize(3);
    points_[i][0]= point_array[i][0];
    points_[i][1]= point_array[i][1];
    points_[i][2]= point_array[i][2];
  }

  return GP_OK;
}





//! Draws the current content of hull_vertices and hull_faces.
//! NB: The colors are set inside the function.
//! \param wireframe sets display mode (wireframe/solid)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull3D::draw(bool wireframe)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull3D::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j, k, k1, k2;
//   double s= 0.1;
  std::vector<double> normal, center;
  double color[4];

  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT);

  glDisable(GL_LIGHTING);

  glLineWidth(3);
  glColor3f(0, 1, 0);

  for(i=0; i<hull_faces.size(); i++)
  {
    glBegin(GL_LINE_LOOP);
     for(j=0; j<hull_faces[i].nbVertices(); j++)
     {
       k= hull_faces[i][j];
       glVertex3f(points_[k][0], points_[k][1], points_[k][2]);
     }
     glEnd();
  }
     // draw face normals:
//   glColor3f(1, 0, 1);
//   glBegin(GL_LINES);
//    for(i=0; i<hull_faces.size(); i++)
//    {
//      normal= hull_faces[i].normal();
//      center= hull_faces[i].center();
// 
//      glVertex3f(center[0], center[1], center[2]);
//      glVertex3f(center[0] + normal[0], center[1] + normal[1], center[2] + normal[2]);
//    }
//   glEnd();


//   glColor3f(1, 0, 1);
//   glBegin(GL_LINES);
//    for(i=0; i<hull_faces.size(); i++)
//    {
//      normal= hull_faces[i].normal();
//      for(j=0; j<hull_faces[i].nbVertices(); j++)
//      {
//        k= hull_faces[i][j];
//        glVertex3f(points_[k][0], points_[k][1], points_[k][2]);
//        glVertex3f(points_[k][0] + normal[0], points_[k][1] + normal[1], points_[k][2] + normal[2]);
//      }
//    }
//   glEnd();


  glColor3f(1, 0, 0);
  glPointSize(6);
  glBegin(GL_POINTS);
   for(i=0; i<hull_vertices.size(); i++)
   {
     glVertex3f(points_[hull_vertices[i]][0], points_[hull_vertices[i]][1], points_[hull_vertices[i]][2]);
   }
  glEnd();

  glColor3f(1, 0, 1);
  glPointSize(6);
  glBegin(GL_POINTS);
   for(i=0; i<points_.size(); i++)
   {
     glVertex3f(points_[i][0], points_[i][1], points_[i][2]);
   }
  glEnd();

  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);

  if(!wireframe)
  {
      g3d_set_color(Blue, NULL);
      for(i=0; i<hull_faces.size(); i++)
      {
        glBegin(GL_POLYGON);
          glNormal3f(hull_faces[i].normal().at(0), hull_faces[i].normal().at(1), hull_faces[i].normal().at(2));
          for(j=0; j<hull_faces[i].nbVertices(); j++)
          {
            k= hull_faces[i][j];
            glVertex3f(points_[k][0], points_[k][1], points_[k][2]);
          }
        glEnd();
      }
  }


  glLineWidth(1);
  glEnable(GL_LIGHTING);

  for(i=0; i<voronoi_cells_.size(); ++i)
  {
    g3d_rgb_from_int(i, color);
    glColor3dv(color);

    for(j=0; j<voronoi_cells_[i].ridges_.size(); ++j)
    {
      k1= voronoi_cells_[i].ridges_[j];

      if(voronoi_cells_[i].ccw_.at(j)) 
      {
        glNormal3f(voronoi_ridges_.at(k1).normal_.at(0), voronoi_ridges_.at(k1).normal_.at(1), voronoi_ridges_.at(k1).normal_.at(2));
        glBegin(GL_POLYGON);
            for(k=0; k<voronoi_ridges_.at(k1).vertices_.size(); ++k)
            {
              k2= voronoi_ridges_.at(k1).vertices_.at(k);
              glVertex3f(voronoi_vertices_.at(k2)[0], voronoi_vertices_.at(k2)[1], voronoi_vertices_.at(k2)[2]);
            }
        glEnd();
      }
      else
      {
        glNormal3f(-voronoi_ridges_.at(k1).normal_.at(0), -voronoi_ridges_.at(k1).normal_.at(1), -voronoi_ridges_.at(k1).normal_.at(2));
        glBegin(GL_POLYGON);
            for(k=0; k<voronoi_ridges_.at(k1).vertices_.size(); ++k)
            {
              k2= voronoi_ridges_.at(k1).vertices_.at(voronoi_ridges_.at(k1).vertices_.size()-1-k);
              glVertex3f(voronoi_vertices_.at(k2)[0], voronoi_vertices_.at(k2)[1], voronoi_vertices_.at(k2)[2]);
            }
        glEnd();
      }

//       if(voronoi_cells_[i].ccw_.at(j))
//       {
//         glBegin(GL_TRIANGLE_FAN);
//         glNormal3f(voronoi_ridges_.at(k1).normal_.at(0), voronoi_ridges_.at(k1).normal_.at(1), voronoi_ridges_.at(k1).normal_.at(2));
//         glNormal3f(0.0,0.0,1.0);
//         glVertex3f(voronoi_ridges_.at(k1).center_.at(0), voronoi_ridges_.at(k1).center_.at(1), voronoi_ridges_.at(k1).center_.at(2));
//         for(k=0; k<voronoi_ridges_.at(k1).vertices_.size(); ++k)
//         {
//           k2= voronoi_ridges_.at(k1).vertices_.at(k);
//           glVertex3f(voronoi_vertices_.at(k2).at(0), voronoi_vertices_.at(k2).at(1), voronoi_vertices_.at(k2).at(2));
//         }
//         k2= voronoi_ridges_.at(k1).vertices_.at(0);
//         glVertex3f(voronoi_vertices_.at(k2).at(0), voronoi_vertices_.at(k2).at(1), voronoi_vertices_.at(k2).at(2));
//         glEnd();
//       }
//       else
//       {
//         glBegin(GL_TRIANGLE_FAN);
//         glNormal3f(-voronoi_ridges_.at(k1).normal_.at(0), -voronoi_ridges_.at(k1).normal_.at(1), -voronoi_ridges_.at(k1).normal_.at(2));
//         glVertex3f(voronoi_ridges_.at(k1).center_.at(0), voronoi_ridges_.at(k1).center_.at(1), voronoi_ridges_.at(k1).center_.at(2));
//         for(k=0; k<voronoi_ridges_.at(k1).vertices_.size(); ++k)
//         {
//           k2= voronoi_ridges_.at(k1).vertices_.at(voronoi_ridges_.at(k1).vertices_.size()-1-k);
//           glVertex3f(voronoi_vertices_.at(k2).at(0), voronoi_vertices_.at(k2).at(1), voronoi_vertices_.at(k2).at(2));
//         }
//         k2= voronoi_ridges_.at(k1).vertices_.back();
//         glVertex3f(voronoi_vertices_.at(k2).at(0), voronoi_vertices_.at(k2).at(1), voronoi_vertices_.at(k2).at(2));
//         glEnd();
//       }

    }

  }

  glPopAttrib();

  return GP_OK;
}


//! Draws the hull face of given index.
//! NB: The colors are set inside the function.
//! \param face_index the index of the face in the face array of the calling gpConvexHull3D
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpConvexHull3D::drawFace(unsigned int face_index)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull3D::drawFace(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(face_index > hull_faces.size())
  {
    printf("%s: %d: gpConvexHull3D::drawFace(): input face index (%d) exceeds the hull's number of faces (%d).\n",__FILE__,__LINE__,face_index,hull_faces.size());
    return GP_ERROR;
  }

  unsigned int i, k;
  double d= 0.01;
  std::vector<double> normal, center;

  normal= hull_faces[face_index].normal();
  center= hull_faces[face_index].center();

  g3d_set_color(Red, NULL);

  glBegin(GL_POLYGON);
    glNormal3f(normal[0], normal[1], normal[2]);
    for(i=0; i<hull_faces[face_index].nbVertices(); i++)
    {
      k= hull_faces[face_index][i];
      glVertex3f(points_[k][0], points_[k][1], points_[k][2]);
    }
  glEnd();

  glBegin(GL_LINES);
    glVertex3f(center[0], center[1], center[2]);
    glVertex3f(center[0] + d*normal[0], center[1] + d*normal[1], center[2] + d*normal[2]);
  glEnd();

  return GP_OK;
}


//! Computes the voronoi regions of the point set stored in the calling gpConvexHull3D variable.
//! Compared to the version of the parent class, it just adds the computation of the ridge normals and how
//! the ridge vertices are ordered.
int gpConvexHull3D::voronoi(bool verbose)
{
  if(this==NULL)
  {
    printf("%s: %d: gpConvexHull3D::voronoi(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j, k, rindex;
  double norm, dot;
  std::vector<double> p1(3), p2(3), op1(3), op2(3), direction(3);

  gpConvexHull::voronoi(verbose);
 
  // compute the normal of each voronoi ridge:
  for(i=0; i<voronoi_ridges_.size(); ++i)
  {
    if( voronoi_ridges_.at(i).site1_id_==UINT_MAX && voronoi_ridges_.at(i).site2_id_==UINT_MAX )
    {   continue;    }

    p1= voronoi_vertices_.at(voronoi_ridges_.at(i).vertices_.at(0));
    p2= voronoi_vertices_.at(voronoi_ridges_.at(i).vertices_.at(1));

    for(k=0; k<op1.size(); ++k)
    {  
      op1.at(k)= p1.at(k) - voronoi_ridges_.at(i).center_.at(k); 
      op2.at(k)= p2.at(k) - voronoi_ridges_.at(i).center_.at(k); 
    }

    voronoi_ridges_.at(i).normal_.resize(3);

    voronoi_ridges_.at(i).normal_.at(0)= op1.at(1)*op2.at(2) - op1.at(2)*op2.at(1);
    voronoi_ridges_.at(i).normal_.at(1)= op1.at(2)*op2.at(0) - op1.at(0)*op2.at(2);
    voronoi_ridges_.at(i).normal_.at(2)= op1.at(0)*op2.at(1) - op1.at(1)*op2.at(0);

    norm= 0.0;
    for(k=0; k<voronoi_ridges_.at(i).normal_.size(); ++k)
    {   norm+= voronoi_ridges_.at(i).normal_.at(k)*voronoi_ridges_.at(i).normal_.at(k);    }
    norm= sqrt(norm);
    for(k=0; k<voronoi_ridges_.at(i).normal_.size(); ++k)
    {   voronoi_ridges_.at(i).normal_.at(k) /= norm;    }

    if( isnan(voronoi_ridges_.at(i).normal_.at(0)) || isnan(voronoi_ridges_.at(i).normal_.at(1)) || isnan(voronoi_ridges_.at(i).normal_.at(2)) )
    {
      printf("ridge #%d, %d vertices\n",i,voronoi_ridges_.at(i).vertices_.size());
      printf("p1 %f %f %f\n", p1.at(0),p1.at(1),p1.at(2));
      printf("p2 %f %f %f\n", p2.at(0),p2.at(1),p2.at(2));
      printf("op1 %f %f %f\n", op1.at(0),op1.at(1),op1.at(2));
      printf("op2 %f %f %f\n", op2.at(0),op2.at(1),op2.at(2));
      printf("norm= %f\n",norm);
      
      printf("normal %f %f %f\n", voronoi_ridges_.at(i).normal_.at(0),voronoi_ridges_.at(i).normal_.at(1),voronoi_ridges_.at(i).normal_.at(2));
    }
 }

  // for each ridge of each voronoi cell, test wether or not the ridge's normal is pointing to the cell site:
  for(i=0; i<voronoi_cells_.size(); ++i)
  {
    voronoi_cells_.at(i).ccw_.resize(voronoi_cells_.at(i).ridges_.size());

    for(j=0; j<voronoi_cells_.at(i).ridges_.size(); ++j)
    {
      rindex= voronoi_cells_.at(i).ridges_.at(j);

      dot= 0.0;
      for(k=0; k<direction.size(); ++k)
      {  
        direction.at(k)= voronoi_ridges_.at(rindex).center_.at(k) - points_.at(voronoi_cells_.at(i).site_id_).at(k); 
        dot+= direction.at(k)*voronoi_ridges_.at(rindex).normal_.at(k);
      }

      voronoi_cells_.at(i).ccw_.at(j)= (dot > 0);
    }

  }


  return GP_OK;
}

//! Initializes the input point set from the given array (with the same order so that, after hull computation,
//! indices in hull_vertices and hull_faces correspond to the indices in point_array).
gpConvexHull6D::gpConvexHull6D(double (*point_array)[6], unsigned int nb_points)
{
  unsigned int i;

  dimension_= 6;
  up_to_date_= false;
  largest_ball_radius_= 0.0;

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

  points_.resize(nb_points);
 
  for(i=0; i<nb_points; i++)
  {
    points_[i].resize(6);
    points_[i][0]= point_array[i][0];
    points_[i][1]= point_array[i][1];
    points_[i][2]= point_array[i][2];
    points_[i][3]= point_array[i][3];
    points_[i][4]= point_array[i][4];
    points_[i][5]= point_array[i][5];
  }

}


//! \ingroup convexHull 
//! Computes sample points inside the convex hull of the vertices of a p3d_polyhedre.
//! A grid is first computed inside an axis-aligned bounding box of the polyhedron.
//! All the points that are outside the convex hull are then removed.
//! The function also computes the distance from each sample to the hull (the distance value
//! is copied in the cost field of the samples)
//! \param polyhedron pointer to the p3d_polyhedre
//! \param step resolution of the grid 
//! \param samples the computed number of samples
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSample_polyhedron_convex_hull(p3d_polyhedre *polyhedron, double step, std::vector<gpVector3D> &samples)
{ 
   if(polyhedron==NULL)
   {
     printf("%s: %d: gpSample_polyhedron_convex_hull(): input p3d_polyhedre is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }

   bool inside;
   unsigned int i, j, k, nx, ny, nz;
   int result;
   double x, y, z, xmin, xmax, ymin, ymax, zmin, zmax, distance;
   std::vector<double> p;
   std::list< std::vector<double> > sampleList;
   std::list< std::vector<double> >::iterator iter;
   std::list<double> distanceList;

   gpConvexHull3D hull(polyhedron->the_points, polyhedron->nb_points);
  
   result= hull.compute(false, -1, false);

   if(result==GP_ERROR)
   {
     printf("%s: %d: gpSample_polyhedron_convex_hull(): error in convex hull computation.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }

   xmin= xmax= polyhedron->the_points[0][0];
   ymin= ymax= polyhedron->the_points[0][1];
   zmin= zmax= polyhedron->the_points[0][2];

   for(i=1; i<polyhedron->nb_points; ++i)
   {
     x= 1.1*polyhedron->the_points[i][0];
     y= 1.1*polyhedron->the_points[i][1];
     z= 1.1*polyhedron->the_points[i][2];

     if(x < xmin) {  xmin= x;  }
     if(x > xmax) {  xmax= x;  }
     if(y < ymin) {  ymin= y;  }
     if(y > ymax) {  ymax= y;  }
     if(z < zmin) {  zmin= z;  }
     if(z > zmax) {  zmax= z;  }
   }
   
   nx= (unsigned int) ((xmax - xmin)/step);
   ny= (unsigned int) ((ymax - ymin)/step);
   nz= (unsigned int) ((zmax - zmin)/step);

   p.resize(3);
   for(i=0; i<=nx; ++i)
   {
      for(j=0; j<=ny; ++j)
      {
          for(k=0; k<=nz; ++k)
          {
            p.at(0)= xmin + i*step;
            p.at(1)= ymin + j*step; 
            p.at(2)= zmin + k*step;
            sampleList.push_back(p);
          }
      }
   }

   iter= sampleList.begin(); 
   while(iter!=sampleList.end())
   {
     result= hull.isPointInside(*iter, inside, distance);
     if(!inside)
     {
       iter= sampleList.erase(iter);
       continue;
     }
     distanceList.push_back(distance);
     iter++;
   }

   samples.resize(sampleList.size());

   for(i=0; i<samples.size(); ++i)
   {
     p= sampleList.front();
     sampleList.pop_front();
     distance= distanceList.front();
     distanceList.pop_front();

     samples[i].x= p.at(0);
     samples[i].y= p.at(1);
     samples[i].z= p.at(2);
     samples[i].cost= distance;
   }

   return GP_OK;
}



