
#ifndef GPKDTREE_H
#define GPKDTREE_H

class gpAABB
{
  private:
    class gpKdTree *tree_;
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;
    double dimX_, dimY_, dimZ_;
    bool root_, leaf_;
    double minimum_size_;
    unsigned int level_;
    class gpAABB *children_[2];
    class gpAABB *brother_;
    std::list<unsigned int> inner_points_;
  public:
    gpAABB(gpKdTree *tree, std::list<unsigned int> &inner_points);
    gpAABB(gpAABB *previous, std::list<unsigned int> &inner_points);
    int divide();
    int draw(unsigned int level);
    int sphereIntersection(p3d_vector3 center, double radius, std::list<gpContact> &points);
};


class gpKdTree
{
 private:
  class gpAABB *root_;

  friend class gpAABB; 
  std::vector<gpContact> points;
  std::list<class gpAABB*> tree;

 public:
  gpKdTree(std::list<gpContact> &contactList);
  int draw(unsigned int level);
  int sphereIntersection(p3d_vector3 center, double radius, std::list<gpContact> &points);
};


#endif

