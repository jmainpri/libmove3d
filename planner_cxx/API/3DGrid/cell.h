#ifndef CELL_HPP
#define CELL_HPP

#include <vector>

class Grid;

class Cell
{
public:
    Cell();
    Cell(int i, std::vector<double> corner, Grid* grid);
    virtual ~Cell();

    bool isInsideCell(std::vector<double> point);
    std::vector<double> getCenter();
    std::vector<double> getCorner() { return _corner; }
    int getIndex() { return _index; }

protected:
    int _index;
    std::vector<double> _corner;
    Grid* _grid;
};

#endif // CELL_HPP
