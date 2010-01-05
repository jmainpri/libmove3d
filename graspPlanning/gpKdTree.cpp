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


//! Divides an AABB along its larger dimension.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABB::divide()
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::divide(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

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

  children_[0]= new gpAABB(this, inner_points1);
  children_[1]= new gpAABB(this, inner_points2);

  children_[0]->divide();
  children_[1]->divide();

  return GP_OK;
}

//! Draws a gpAABB if its level is less or equal to the input value.
//! \param level the value to compare to the gpAABB level
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpAABB::draw(unsigned int level)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  glPushAttrib(GL_LIGHTING | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glPointSize(4);

  if( !leaf_ && level_==level )
  {
    glColor3f(0.0, 1.0, 0.0);
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


int gpAABB::sphereIntersection(p3d_vector3 center, double radius, std::list<gpContact> &points)
{
  if(this==NULL)
  {
    printf("%s: %d: gpAABB::sphereIntersection(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

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
//     points.push_back(inner_points_.front());
    return GP_OK;
  }

  return GP_OK;
}


//! Constructor of the class gpKdTree.
//! \param contactList a list of contact (only their positions will be used to build the Kd tree)
gpKdTree::gpKdTree(std::list<gpContact> &contactList)
{
  unsigned int i, nb_points= contactList.size();
  std::list<gpContact>::iterator iter;
  std::list<unsigned int> inner_points;

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


//! Draws a gpKdTree at the given level.
//! \param level the only AABB that will be displayed are the ones with a level less or equal than this parameter value
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

  glPushAttrib(GL_POINT_BIT);
  glPointSize(2);
  glBegin(GL_POINTS);
  for(i=0; i<points.size(); ++i)
  {
    glVertex3dv(points[i].position);
  }
  glEnd();
  glPopAttrib();

  root_->draw(level);

  return GP_OK;
}



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

