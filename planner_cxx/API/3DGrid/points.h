#ifndef POINTS_H
#define POINTS_H

#include "../../../other_libraries/Eigen/Core"
#include <vector>

USING_PART_OF_NAMESPACE_EIGEN

class Points
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Points();

    void push_back(Vector3d point);
    void drawAllPoints();
    void drawOnePoint(int i);

private:
    std::vector< Vector3d > _AllPoints;
    Vector3d _CubeSize;
};

extern Points* PointsToDraw;

#endif // POINTS_H
