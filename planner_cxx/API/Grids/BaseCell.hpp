#ifndef BASECELL_HPP
#define BASECELL_HPP

#include "../../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

/**
  * Base class for a Grid
  */
namespace API
{
    class BaseCell
    {
    public:
        BaseCell();
        virtual ~BaseCell();
    };
}

#endif // BASECELL_HPP
