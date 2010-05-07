#ifndef CELL_HPP
#define CELL_HPP

#include <Eigen/Core>

#include "BaseCell.hpp"

#include <libxml/parser.h>

USING_PART_OF_NAMESPACE_EIGEN

/**
  * @ingroup CPP_API
  * @defgroup GRID Grid over the WS
  */

/**
  @ingroup GRID
  */
namespace API
{
    class ThreeDGrid;
	

    class ThreeDCell : public BaseCell
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ThreeDCell();
		ThreeDCell(int i, ThreeDGrid* grid);
        ThreeDCell(int i, Vector3d corner, ThreeDGrid* grid);
        virtual ~ThreeDCell();
		
		virtual double getCost() { return 0; };

        bool isInsideCell(Vector3d point);

        Vector3d getCenter();
        Vector3d getCorner() { return _corner; }
        Vector3d getRandomPoint();
        Vector3d getCellSize();

        int getIndex() { return _index; }

		void setCorner(const Vector3d& corner) { _corner = corner; }
		
        virtual void draw();
		
		void writeToXml(xmlNodePtr _XmlCellNode_);
		bool readCellFromXml(xmlNodePtr _XmlCellNode_);

        bool operator==( ThreeDCell otherCell) { return ((otherCell._index) == (this->_index)); }

    protected:
        int _index;
        Vector3d _corner;
        ThreeDGrid* _grid;
    };

}
#endif // CELL_HPP
