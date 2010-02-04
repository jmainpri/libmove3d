#ifndef BASEGRID_HPP
#define BASEGRID_HPP

/**
  * Base class for a Grid
  */
namespace API
{
    class BaseGrid
    {
    public:
        BaseGrid();
        virtual ~BaseGrid();

        virtual void draw() =0;
    };
}

#endif // BASEGRID_HPP
