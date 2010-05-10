#ifndef POINTS_H
#define POINTS_H

#include <Eigen/Core>
#include <vector>

USING_PART_OF_NAMESPACE_EIGEN

/**
@ingroup GRID
@brief vector of 3d points that can be ploted in the 3d viewer as cubes very fast
*/
class ThreeDPoints
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ThreeDPoints();

    void push_back(Vector3d point);
    void drawAllPoints();
    void drawOnePoint(int i);

private:
    std::vector< Vector3d > _AllPoints;
    Vector3d _CubeSize;
};

extern ThreeDPoints* PointsToDraw;

#endif // POINTS_H
