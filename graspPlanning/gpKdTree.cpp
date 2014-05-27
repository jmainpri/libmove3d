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
#include "GraspPlanning-pkg.h"
#include <list>


//! Constructor for root gpAABB.
//! \param tree pointer to the Kd tree, the AABB will belong to
//! \param inner_points list of the points the AABB must bound
gpAABB::gpAABB(gpKdTree *tree, std::list<unsigned int> &inner_points)
{
  double x, y, z;
  std::list<unsigned int>::iterator iter;

  tree_= tree;
  root_= true;
  leaf_= true;
  level_= 1;
  tree_->depth_= 1;

  children_[0]= NULL;
  children_[1]= NULL;
  brother_= NULL;

  inner_points_= inner_points;

  for(iter=inner_points_.begin(); iter!=inner_points_.end(); iter++)
  {
    x= tree->points.at(*iter).position[0];
    y= tree->points.at(*iter).position[1];
    z= tree->points.at(*iter).position[2];

    if(iter==inner_points_.begin())
    {
      xmin_= xmax_= x;
      ymin_= ymax_= y;
      zmin_= zmax_= z;
    }
    else
    {
      if(x < xmin_)  {  xmin_= x;  }
      if(x > xmax_)  {  xmax_= x;  }
      if(y < ymin_)  {  ymin_= y;  }
      if(y > ymax_)  {  ymax_= y;  }
      if(z < zmin_)  {  zmin_= z;  }
      if(z > zmax_)  {  zmax_= z;  }
    }
  }
}

//! Constructor for non root gpAABB.
//! \param previous pointer to the previous AABB in the tree
//! \param inner_points list of the points the AABB must bound
gpAABB::gpAABB(gpAABB *previous, std::list<unsigned int> &inner_points)
{
  double x, y, z;
  std::list<unsigned int>::iterator iter;

  if(previous==NULL)
  {  
    printf("%s: %d: gpAABB::gpAABB(): previous is NULL.\n",__FILE__,__LINE__);   
    return;
  }
  
  previous->leaf_= false;
  tree_= previous->tree_;
  root_= false;
  leaf_= true;
  level_= previous->level_ + 1;
  if(level_ > tree_->depth_) 
  {  tree_->depth_= level_;  }

  children_[0]= NULL;
  children_[1]= NULL;

  inner_points_= inner_points;

  for(iter=inner_points_.begin(); iter!=inner_points_.end(); iter++)
  {
    x= tree_->points.at(*iter).position[0];
    y= tree_->points.at(*iter).position[1];
    z= tree_->points.at(*iter).position[2];

    if(iter==inner_points_.begin())
    {
      xmin_= xmax_= x;
      ymin_= ymax_= y;
      zmin_= zmax_= z;
    }
    else
    {
      if(x < xmin_)  {  xmin_= x;  }
      if(x > xmax_)  {  xmax_= x;  }
      if(y < ymin_)  {  ymin_= y;  }
      if(y > ymax_)  {  ymax_= y;  }
      if(z < zmin_)  {  zmin_= z;  }
      if(z > zmax_)  {  zmax_= z;  }
    }
  }
}

gpAABB::~gpAABB()
{
  gpAABB *aabb1, *aabb2;
  
  aabb1= children_[0];
  aabb2= children_[1];

  if(aabb1!=NULL) 
  {  delete aabb1;  }

  if(aabb2!=NULL) 
  {  delete aabb2;  }
}


//! Divides an AABB along its larger dimension.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABB::divide()
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::divide(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  /*
printf("divide\n");
if(level_>10) {
	 leaf_= true;  return GP_OK;
}*/
	
  double dimX, dimY, dimZ, mid;
  gpContact contact;
  std::list<unsigned int> inner_points1, inner_points2;
  std::list<unsigned int>::iterator iter;

  dimX= xmax_ - xmin_;
  dimY= ymax_ - ymin_;
  dimZ= zmax_ - zmin_;

  if(inner_points_.size()==1)
  {
    leaf_= true; 
    xmin_= xmax_= tree_->points.at(inner_points_.front()).position[0];
    ymin_= ymax_= tree_->points.at(inner_points_.front()).position[1];
    zmin_= zmax_= tree_->points.at(inner_points_.front()).position[2];
    return GP_OK;
  }

  if( (dimX > dimY) && (dimX > dimZ) )
  {
    mid= xmin_ + 0.5*dimX;
    for(iter=inner_points_.begin(); iter!=inner_points_.end(); iter++)
    {
      if(tree_->points.at(*iter).position[0] < mid)
      {
        inner_points1.push_back(*iter);
      }
      else
      {
        inner_points2.push_back(*iter);
      }
    }
  } 
  else if(dimY > dimZ)
  {
    mid= ymin_ + 0.5*dimY;
    for(iter=inner_points_.begin(); iter!=inner_points_.end(); iter++)
    {
      if(tree_->points.at(*iter).position[1] < mid)
      {
        inner_points1.push_back(*iter);
      }
      else
      {
        inner_points2.push_back(*iter);
      }
    }
  }
  else
  {
    mid= zmin_ + 0.5*dimZ;
    for(iter=inner_points_.begin(); iter!=inner_points_.end(); iter++)
    {
      if(tree_->points.at(*iter).position[2] < mid)
      {
        inner_points1.push_back(*iter);
      }
      else
      {
        inner_points2.push_back(*iter);
      }
    }
  }

  // if one of the two new point sets is void, the original point set
  // is too small or flat to be broken in two -> we stop the division
  if(inner_points1.empty() || inner_points2.empty())
  {
	 leaf_= true;
	 return GP_OK;
  }
  
  children_[0]= new gpAABB(this, inner_points1);
  children_[1]= new gpAABB(this, inner_points2);

  children_[0]->divide();
  children_[1]->divide();

  return GP_OK;
}

//! Draws a gpAABB, as a wire box, if its level is equal to the input value or if it is a leaf.
//! \param level the value to compare to the gpAABB level
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABB::draw(unsigned int level)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  glPushAttrib(GL_LIGHTING_BIT | GL_POINT_BIT | GL_LINE_BIT);
  glDisable(GL_LIGHTING);
  glPointSize(4);
  glLineWidth(4);

  // enable blending to draw antialiased lines:
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if( !leaf_ && level_==level )
  {
    glColor4f(0.0, 1.0, 0.0, 0.7);
    glBegin(GL_LINES);
      glVertex3f(xmin_, ymin_, zmin_);
      glVertex3f(xmax_, ymin_, zmin_);

      glVertex3f(xmin_, ymax_, zmin_);
      glVertex3f(xmax_, ymax_, zmin_);

      glVertex3f(xmin_, ymax_, zmax_);
      glVertex3f(xmax_, ymax_, zmax_);

      glVertex3f(xmin_, ymin_, zmax_);
      glVertex3f(xmax_, ymin_, zmax_);

      glVertex3f(xmin_, ymin_, zmin_);
      glVertex3f(xmin_, ymax_, zmin_);

      glVertex3f(xmax_, ymin_, zmin_);
      glVertex3f(xmax_, ymax_, zmin_);

      glVertex3f(xmax_, ymin_, zmax_);
      glVertex3f(xmax_, ymax_, zmax_);

      glVertex3f(xmin_, ymin_, zmax_);
      glVertex3f(xmin_, ymax_, zmax_);

      glVertex3f(xmin_, ymin_, zmin_);
      glVertex3f(xmin_, ymin_, zmax_);

      glVertex3f(xmax_, ymin_, zmin_);
      glVertex3f(xmax_, ymin_, zmax_);

      glVertex3f(xmax_, ymax_, zmin_);
      glVertex3f(xmax_, ymax_, zmax_);

      glVertex3f(xmin_, ymax_, zmin_);
      glVertex3f(xmin_, ymax_, zmax_);
    glEnd();

  }
  if( leaf_ && level>=level_ )
  {
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_POINTS);
      glVertex3d(xmin_, ymin_, zmin_);
    glEnd();
  }
  glPopAttrib();

  if(leaf_ || level_>=level)
  {  return GP_OK;  }
  
  children_[0]->draw(level);
  children_[1]->draw(level);  
 
  return GP_OK;
}


//! Finds all the points belonging to the AABB that are inside a sphere.
//! \param center sphere center
//! \param radius sphere radius
//! \param points points that are inside the sphere
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABB::sphereIntersection(p3d_vector3 center, double radius, std::list<gpContact> &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::sphereIntersection(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  p3d_vector3 d;
  gpContact p;
  std::list<unsigned int>::iterator iter;

  if(!leaf_)
  {
    if( (center[0] + radius < xmin_) || (center[0] - radius > xmax_) ||
        (center[1] + radius < ymin_) || (center[1] - radius > ymax_) ||
        (center[2] + radius < zmin_) || (center[2] - radius > zmax_) )
    {
      return GP_OK;
    }
    else
    {
      if(children_[0]!=NULL)
      {  children_[0]->sphereIntersection(center, radius, points);  }
      if(children_[1]!=NULL)
      {  children_[1]->sphereIntersection(center, radius, points);  }
    }
  }
  else
  {
    p= tree_->points.at(inner_points_.front());
    p3d_vectSub(p.position, center, d);
    if( p3d_vectNorm(d) < radius )
    {
      points.push_back(p);
    }

    return GP_OK;
  }

  return GP_OK;
}


//! Default onstructor of the class gpKdTree.
gpKdTree::gpKdTree()
{
  root_= NULL;
  depth_= 0;
}

//! Constructor of the class gpKdTree.
//! \param contactList a list of contact (only their positions will be used to build the Kd tree)
gpKdTree::gpKdTree(std::list<gpContact> &contactList)
{
  unsigned int i, nb_points= contactList.size();
  std::list<gpContact>::iterator iter;
  std::list<unsigned int> inner_points;

  depth_= 0;
  points.resize(nb_points);
    
  i= 0;
  for(iter=contactList.begin(); iter!=contactList.end(); iter++)
  {
    points.at(i)= (*iter);
    inner_points.push_back(i);
    i++;
  }

  root_= new gpAABB(this, inner_points);

  root_->divide();
}

gpKdTree::~gpKdTree()
{
  if(root_!=NULL)
  {  delete root_;  }
}

//! Sets the input points of the tree and builds it.
//! \param contactList a list of contact (only their positions will be used to build the Kd tree)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpKdTree::build(std::list<gpContact> &contactList)
{
  if(this==NULL)
  {
    printf("%s: %d: gpKdTree::build(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, nb_points= contactList.size();
  std::list<gpContact>::iterator iter;
  std::list<unsigned int> inner_points;

  depth_= 0;
  points.resize(nb_points);
    
  i= 0;
  for(iter=contactList.begin(); iter!=contactList.end(); iter++)
  {
    points.at(i)= (*iter);
    inner_points.push_back(i);
    i++;
  }

  if(root_!=NULL)
  {
    delete root_;
    root_= NULL;
  }

  root_= new gpAABB(this, inner_points);

  root_->divide();

  return GP_OK;
}



//! Draws a gpKdTree at the given level.
//! \param level the only AABB that will be displayed are the ones
//! with a level less or equal than this parameter value
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpKdTree::draw(unsigned int level)
{
  if(this==NULL)
  {
    printf("%s: %d: gpKdTree::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(root_==NULL)
  {
    return GP_ERROR;
  }

  unsigned int i;

  root_->draw(level);

  glPushAttrib(GL_POINT_BIT);
  glPointSize(3);

  glColor3f(0.0, 0.0, 1.0);

  glBegin(GL_POINTS);
  for(i=0; i<points.size(); ++i)
  {
    glVertex3dv(points[i].position);
  }
  glEnd();

  glPopAttrib();

  return GP_OK;
}


//! Finds all the points of the tree that are inside a sphere.
//! \param center sphere center
//! \param radius sphere radius
//! \param points points that are inside the sphere
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpKdTree::sphereIntersection(p3d_vector3 center, double radius, std::list<gpContact> &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpKdTree::sphereIntersection(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }


  if(root_!=NULL)
  {
    root_->sphereIntersection(center, radius, points);
  }

  return GP_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


//! Constructor for root gpAABBTris.
//! \param tree pointer to the Kd tree, the AABB will belong to
//! \param inner_points list of the points the AABB must bound
gpAABBTris::gpAABBTris(gpKdTreeTris *tree, std::list<unsigned int> &inner_triangles)
{
  unsigned int index1, index2, index3;
  double xmin, ymin, zmin, xmax, ymax, zmax; 
  double x1, y1, z1, x2, y2, z2, x3, y3, z3;
  std::list<unsigned int>::iterator iter;

  tree_= tree;
  root_= true;
  leaf_= true;
  inside_= false;
  level_= 1;
  tree_->depth_= 1;

  children_[0]= NULL;
  children_[1]= NULL;
  brother_= NULL;

  inner_triangles_= inner_triangles;

  for(iter=inner_triangles_.begin(); iter!=inner_triangles_.end(); iter++)
  {
    index1= tree_->polyhedron_->the_faces[*iter].the_indexs_points[0] - 1;
    index2= tree_->polyhedron_->the_faces[*iter].the_indexs_points[1] - 1;
    index3= tree_->polyhedron_->the_faces[*iter].the_indexs_points[2] - 1;

    x1= tree_->polyhedron_->the_points[index1][0];
    y1= tree_->polyhedron_->the_points[index1][1];
    z1= tree_->polyhedron_->the_points[index1][2];

    x2= tree_->polyhedron_->the_points[index2][0];
    y2= tree_->polyhedron_->the_points[index2][1];
    z2= tree_->polyhedron_->the_points[index2][2];

    x3= tree_->polyhedron_->the_points[index3][0];
    y3= tree_->polyhedron_->the_points[index3][1];
    z3= tree_->polyhedron_->the_points[index3][2];

    xmin= MIN( x1, MIN(x2, x3) );
    ymin= MIN( y1, MIN(y2, y3) );
    zmin= MIN( z1, MIN(z2, z3) );
    xmax= MAX( x1, MAX(x2, x3) );
    ymax= MAX( y1, MAX(y2, y3) );
    zmax= MAX( z1, MAX(z2, z3) );

    if(iter==inner_triangles_.begin())
    {
      xmin_= xmin;
      xmax_= xmax;
      ymin_= ymin;
      ymax_= ymax;
      zmin_= zmin;
      zmax_= zmax;
    }
    else
    {
      if(xmin < xmin_)  {  xmin_= xmin;  }
      if(xmax > xmax_)  {  xmax_= xmax;  }
      if(ymin < ymin_)  {  ymin_= ymin;  }
      if(ymax > ymax_)  {  ymax_= ymax;  }
      if(zmin < zmin_)  {  zmin_= zmin;  }
      if(zmax > zmax_)  {  zmax_= zmax;  }
    }
  }
}

//! Constructor for non root gpAABBTris.
//! \param previous pointer to the previous AABB in the tree
//! \param xmin minimal coordinate of the AABB along x axis
//! \param xmax maximal coordinate of the AABB along x axis
//! \param ymin minimal coordinate of the AABB along y axis
//! \param ymax maximal coordinate of the AABB along y axis
//! \param zmin minimal coordinate of the AABB along z axis
//! \param zmax maximal coordinate of the AABB along z axis
//! \param inner_points list of the points the AABB must bound
gpAABBTris::gpAABBTris(gpAABBTris *previous, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  unsigned int index1, index2, index3;
  std::list<unsigned int>::iterator iter;

  if(previous==NULL)
  {  
    printf("%s: %d: gpAABBTris::gpAABBTris(): previous is NULL.\n",__FILE__,__LINE__);   
    return;
  }
  
  previous->leaf_= false;
  tree_= previous->tree_;
  root_= false;
  leaf_= true;
  inside_= false;
  level_= previous->level_ + 1;
  if(level_ > tree_->depth_) 
  {  tree_->depth_= level_;  }

  children_[0]= NULL;
  children_[1]= NULL;

  xmin_= xmin;
  xmax_= xmax;
  ymin_= ymin;
  ymax_= ymax;
  zmin_= zmin;
  zmax_= zmax;

  inner_triangles_= previous->inner_triangles_;

  iter= inner_triangles_.begin();
  while(iter!=inner_triangles_.end())
  {
    index1= tree_->polyhedron_->the_faces[*iter].the_indexs_points[0] - 1;
    index2= tree_->polyhedron_->the_faces[*iter].the_indexs_points[1] - 1;
    index3= tree_->polyhedron_->the_faces[*iter].the_indexs_points[2] - 1;

    if(isTriangleOutside(tree_->polyhedron_->the_points[index1], tree_->polyhedron_->the_points[index2], tree_->polyhedron_->the_points[index3]))
    {
       iter= inner_triangles_.erase(iter);
       continue;
    }
    iter++;
  }
}

//! Tests if a triangle is completely outside the AABB.
//! \param p1 first vertex of the triangle
//! \param p2 second vertex of the triangle
//! \param p3 third vertex of the triangle
//! \return true if the triangle is completely outside the AABB, false otherwise
bool gpAABBTris::isTriangleOutside(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
  unsigned int i;
  double d, r[8];
  p3d_vector3 p1p2, p1p3, n, p[8];

  if( (p1[0] < xmin_) && (p2[0] < xmin_) && (p3[0] < xmin_) )
  {   return true;   }
  if( (p1[0] > xmax_) && (p2[0] > xmax_) && (p3[0] > xmax_) )
  {   return true;   }

  if( (p1[1] < ymin_) && (p2[1] < ymin_) && (p3[1] < ymin_) )
  {   return true;   }
  if( (p1[1] > ymax_) && (p2[1] > ymax_) && (p3[1] > ymax_) )
  {   return true;   }

  if( (p1[2] < zmin_) && (p2[2] < zmin_) && (p3[2] < zmin_) )
  {   return true;   }
  if( (p1[2] > zmax_) && (p2[2] > zmax_) && (p3[2] > zmax_) )
  {   return true;   }

  p3d_vectSub(p2, p1, p1p2);
  p3d_vectSub(p3, p1, p1p3);

  p3d_vectNormalize(p1p2, p1p2);
  p3d_vectNormalize(p1p3, p1p3);
  p3d_vectXprod(p1p2, p1p3, n);
  p3d_vectNormalize(n, n);
  d= -p3d_vectDotProd(p1, n);

  p[0][0]=  xmin_;  p[0][1]=  ymin_;   p[0][2]=  zmin_;
  p[1][0]=  xmin_;  p[1][1]=  ymin_;   p[1][2]=  zmax_;
  p[2][0]=  xmin_;  p[2][1]=  ymax_;   p[2][2]=  zmin_;
  p[3][0]=  xmin_;  p[3][1]=  ymax_;   p[3][2]=  zmax_;
  p[4][0]=  xmax_;  p[4][1]=  ymin_;   p[4][2]=  zmin_;
  p[5][0]=  xmax_;  p[5][1]=  ymin_;   p[5][2]=  zmax_;
  p[6][0]=  xmax_;  p[6][1]=  ymax_;   p[6][2]=  zmin_;
  p[7][0]=  xmax_;  p[7][1]=  ymax_;   p[7][2]=  zmax_;


  for(i=0; i<8; ++i)
  {   r[i]= p3d_vectDotProd(p[i], n) + d;    }

  if(r[0]>0)
  {
    if( (r[1]>0) && (r[2]>0) && (r[3]>0) && (r[4]>0) && (r[5]>0) && (r[6]>0) && (r[7]>0) )
    {  return true; }
  }
  else
  {
    if( (r[1]<0) && (r[2]<0) && (r[3]<0) && (r[4]<0) && (r[5]<0) && (r[6]<0) && (r[7]<0) )
    {  return true; }
  }

  return false;
}

//! Tests if the AABB is entirely inside the p3d_polyhedre.
//! \return true if the AABB is completely inside the triangle mesh, false otherwise
bool gpAABBTris::isInsidePolyhedre()
{
  unsigned int i, k;
  unsigned int index1, index2, index3;
  double d, dmin;
  bool inside;
  p3d_vector3 vertex[8], closestPoint, diff;
  p3d_vector3 *points= NULL;
  p3d_face *faces= NULL;

  vertex[0][0]=  xmin_;  vertex[0][1]=  ymin_;   vertex[0][2]=  zmin_;
  vertex[1][0]=  xmin_;  vertex[1][1]=  ymin_;   vertex[1][2]=  zmax_;
  vertex[2][0]=  xmin_;  vertex[2][1]=  ymax_;   vertex[2][2]=  zmin_;
  vertex[3][0]=  xmin_;  vertex[3][1]=  ymax_;   vertex[3][2]=  zmax_;
  vertex[4][0]=  xmax_;  vertex[4][1]=  ymin_;   vertex[4][2]=  zmin_;
  vertex[5][0]=  xmax_;  vertex[5][1]=  ymin_;   vertex[5][2]=  zmax_;
  vertex[6][0]=  xmax_;  vertex[6][1]=  ymax_;   vertex[6][2]=  zmin_;
  vertex[7][0]=  xmax_;  vertex[7][1]=  ymax_;   vertex[7][2]=  zmax_;


  points= tree_->polyhedron_->the_points;
  faces= tree_->polyhedron_->the_faces; 

  for(k=0; k<8; ++k)
  {
    // The following part is a test to know if an AABB vertex is inside the polyhedron.
   // It works but might not be very robust.
    for(i=0; i<tree_->polyhedron_->nb_faces; ++i)
    {
      index1= faces[i].the_indexs_points[0] - 1;
      index2= faces[i].the_indexs_points[1] - 1;
      index3= faces[i].the_indexs_points[2] - 1;
  
      d= gpPoint_to_triangle_distance(vertex[k], points[index1], points[index2], points[index3], closestPoint);

      p3d_vectSub(closestPoint, vertex[k], diff);
      p3d_vectNormalize(diff, diff);
 
      if(d < dmin || i==0)
      {
        dmin= d;
        if(faces[i].plane==NULL)
        { p3d_build_planes(tree_->polyhedron_); }
        else
        {
          if(p3d_vectDotProd(faces[i].plane->normale, diff) > 0)
          {  inside= true; }
          else
          {  inside= false; }
        }

      }
    }
    if(!inside)
    {  return false; }
  }

  return true;
}

//! Divides an AABB along its larger dimension.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABBTris::divide()
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABBTris::divide(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  double dimX, dimY, dimZ;

  dimX= xmax_ - xmin_;
  dimY= ymax_ - ymin_;
  dimZ= zmax_ - zmin_;

  if( (dimX > dimY) && (dimX > dimZ) )
  {
    children_[0]= new gpAABBTris(this, xmin_, xmin_+0.5*dimX, ymin_, ymax_, zmin_, zmax_);
    children_[1]= new gpAABBTris(this, xmin_+0.5*dimX, xmax_, ymin_, ymax_, zmin_, zmax_);
  } 
  else if(dimY > dimZ)
  {
    children_[0]= new gpAABBTris(this, xmin_, xmax_, ymin_, ymin_+0.5*dimY, zmin_, zmax_);
    children_[1]= new gpAABBTris(this, xmin_, xmax_, ymin_+0.5*dimY, ymax_, zmin_, zmax_);
  }
  else
  {
    children_[0]= new gpAABBTris(this, xmin_, xmax_, ymin_, ymax_, zmin_, zmin_+0.5*dimZ);
    children_[1]= new gpAABBTris(this, xmin_, xmax_, ymin_, ymax_, zmin_+0.5*dimZ, zmax_);
  }

  if(children_[0]->inner_triangles_.empty())
  {
    if(children_[0]->isInsidePolyhedre())
    {  children_[0]->inside_= true; }
  }

  if(!children_[0]->inner_triangles_.empty() && MAX(dimX, MAX(dimY, dimZ)) > 0.00 && children_[0]->level_<15)
  {   children_[0]->divide();   }

  if(children_[1]->inner_triangles_.empty())
  {
    if(children_[1]->isInsidePolyhedre())
    {  children_[1]->inside_= true; }
  }

  if(!children_[1]->inner_triangles_.empty() && MAX(dimX, MAX(dimY, dimZ)) > 0.00 && children_[1]->level_<15)
  {   children_[1]->divide();   }

  return GP_OK;
}

//! Draws a gpAABBTris if its level is equal to the input value or if it is a leaf
//! and if it is not ouside the polyhedron interior.
//! \param level the value to compare to the gpAABB level
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABBTris::draw(unsigned int level)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABBTris::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  glPushAttrib(GL_LIGHTING | GL_POINT_BIT);
//   glDisable(GL_LIGHTING);
  glPointSize(4);

  if( (leaf_ || level_==level) && (!inner_triangles_.empty() || inside_) )
  {
    glColor3f(0.0, 1.0, 0.0);
    g3d_set_color(Green, NULL);
//     glBegin(GL_LINES);
//       glVertex3f(xmin_, ymin_, zmin_);
//       glVertex3f(xmax_, ymin_, zmin_);
// 
//       glVertex3f(xmin_, ymax_, zmin_);
//       glVertex3f(xmax_, ymax_, zmin_);
// 
//       glVertex3f(xmin_, ymax_, zmax_);
//       glVertex3f(xmax_, ymax_, zmax_);
// 
//       glVertex3f(xmin_, ymin_, zmax_);
//       glVertex3f(xmax_, ymin_, zmax_);
// 
//       glVertex3f(xmin_, ymin_, zmin_);
//       glVertex3f(xmin_, ymax_, zmin_);
// 
//       glVertex3f(xmax_, ymin_, zmin_);
//       glVertex3f(xmax_, ymax_, zmin_);
// 
//       glVertex3f(xmax_, ymin_, zmax_);
//       glVertex3f(xmax_, ymax_, zmax_);
// 
//       glVertex3f(xmin_, ymin_, zmax_);
//       glVertex3f(xmin_, ymax_, zmax_);
// 
//       glVertex3f(xmin_, ymin_, zmin_);
//       glVertex3f(xmin_, ymin_, zmax_);
// 
//       glVertex3f(xmax_, ymin_, zmin_);
//       glVertex3f(xmax_, ymin_, zmax_);
// 
//       glVertex3f(xmax_, ymax_, zmin_);
//       glVertex3f(xmax_, ymax_, zmax_);
// 
//       glVertex3f(xmin_, ymax_, zmin_);
//       glVertex3f(xmin_, ymax_, zmax_);
//     glEnd();

      glBegin(GL_QUADS);
        glNormal3f(1.0, 0.0, 0.0);
        glVertex3f(xmax_, ymin_, zmin_);
        glVertex3f(xmax_, ymax_, zmin_);
        glVertex3f(xmax_, ymax_, zmax_);
        glVertex3f(xmax_, ymin_, zmax_);

        glNormal3f(-1.0, 0.0, 0.0);
        glVertex3f(xmin_, ymax_, zmin_);
        glVertex3f(xmin_, ymin_, zmin_);
        glVertex3f(xmin_, ymin_, zmax_);
        glVertex3f(xmin_, ymax_, zmax_);

        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(xmax_, ymax_, zmin_);
        glVertex3f(xmin_, ymax_, zmin_);
        glVertex3f(xmin_, ymax_, zmax_);
        glVertex3f(xmax_, ymax_, zmax_);
      
        glNormal3f(0.0, -1.0, 0.0);
        glVertex3f(xmin_, ymin_, zmin_);
        glVertex3f(xmax_, ymin_, zmin_);
        glVertex3f(xmax_, ymin_, zmax_);
        glVertex3f(xmin_, ymin_, zmax_);

        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(xmin_, ymin_, zmax_);
        glVertex3f(xmax_, ymin_, zmax_);
        glVertex3f(xmax_, ymax_, zmax_);
        glVertex3f(xmin_, ymax_, zmax_);

        glNormal3f(0.0, 0.0, -1.0);
        glVertex3f(xmax_, ymin_, zmin_);
        glVertex3f(xmin_, ymin_, zmin_);
        glVertex3f(xmin_, ymax_, zmin_);
        glVertex3f(xmax_, ymax_, zmin_);
      glEnd(); 

  }
  glPopAttrib();

  if(leaf_ || level_>=level)
  {  return GP_OK;  }
  
  children_[0]->draw(level);
  children_[1]->draw(level);
 
  return GP_OK;
}


int gpAABBTris::sample(double step, std::list<gpVector3D> &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABBTris::sample(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j, k, nx, ny, nz;
  double dimX, dimY, dimZ;
  gpVector3D p;
  
  dimX= xmax_ - xmin_;
  dimY= ymax_ - ymin_;
  dimZ= zmax_ - zmin_;

  nx= (unsigned int) (dimX/step);
  ny= (unsigned int) (dimY/step); 
  nz= (unsigned int) (dimZ/step);

  if( leaf_  && (!inner_triangles_.empty() || inside_) )
  {
    for(i=0; i<=nx; ++i)
    {  
      for(j=0; j<=ny; ++j)
      {  
        for(k=0; k<=nz; ++k)
        {  
          p.set(xmin_ + i*step, ymin_ + step, zmin_ + step);
          points.push_back(p);
        }
      }
    }
  }
 
  if(!leaf_)
  {
    children_[0]->sample(step, points);
    children_[1]->sample(step, points);
  }

  return GP_OK;
}


gpKdTreeTris::gpKdTreeTris(p3d_polyhedre *polyhedron)
{
  polyhedron_= NULL;
  if(polyhedron==NULL)
  {
    printf("%s: %d: gpKdTreeTris::gpKdTreeTris(): input p3d_polyhedre is NULL.\n",__FILE__,__LINE__);
    return;
  }
  else
  {
   polyhedron_= polyhedron;
  }

  unsigned int i;

  depth_= 0;

  for(i=0; i<polyhedron_->nb_faces; i++)
  {
    inner_triangles_.push_back(i);
  }

  root_= new gpAABBTris(this, inner_triangles_);

  root_->divide();
}

int gpKdTreeTris::draw(unsigned int level)
{
  if(this==NULL)
  {
    printf("%s: %d: gpKdTreeTris::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(root_==NULL)
  {
    return GP_ERROR;
  }

//   unsigned int i;
//   p3d_vector3 *points= polyhedron_->the_points;

//   p3d_face *faces= polyhedron_->the_faces;

  g3d_set_color(Blue, NULL);
//   glBegin(GL_TRIANGLES);
//    for(i=0; i<polyhedron_->nb_faces; ++i)
//    {
//      glVertex3dv(points[faces[i].the_indexs_points[0]-1]);
//      glVertex3dv(points[faces[i].the_indexs_points[1]-1]);
//      glVertex3dv(points[faces[i].the_indexs_points[2]-1]);
//    }
//   glEnd(); 
//   glPushAttrib(GL_POINT_BIT);
//   glPointSize(2);
//   glBegin(GL_POINTS);
//   for(i=0; i<points.size(); ++i)
//   {
//     glVertex3dv(points[i].position);
//   }
//   glEnd();
//   glPopAttrib();

  root_->draw(level);

  return GP_OK;
}

//! Computes a set of points inside the Kd tree volume.
//! The set of points is obtained by computing a grid inside each of the leaf
//! of the tree except for the leaves that are outside the polyhedron. 
//! \param step discretization step
//! \param points a list containing the computed points
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpKdTreeTris::pointCloud(double step, std::list<gpVector3D> &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpKdTreeTris::pointCloud(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(root_==NULL)
  {
    return GP_ERROR;
  }

  root_->sample(step, points);

  return GP_OK;
}

