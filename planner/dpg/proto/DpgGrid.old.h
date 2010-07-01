#ifndef __DPGGRID_H__
#define __DPGGRID_H__
#include "Planner-pkg.h"
#include "P3d-pkg.h"
#include "DpgCell.h"
#include <vector>

class DpgGrid {
  public:
    //constructors and destructors
    DpgGrid(p3d_env* env);
    virtual ~DpgGrid();

    //setters and getters
    inline int getNbCells(void){return _nbCells;}
    inline int getNbCellsOverX(void){return _nbCellsX;}
    inline int getNbCellsOverY(void){return _nbCellsY;}
    inline int getNbCellsOverZ(void){return _nbCellsZ;}

    //functions
    inline DpgCell* getCellAt(int x, int y, int z);
    //1 = vers les positifs, -1 = vers les negatifs, 0 retourner toutes les cellules
    std::vector<DpgCell*> getCellPoint(double *point, int dx, int dy, int dz);
    void init(void);
    std::vector<DpgCell*> getCellListForObject(p3d_obj* obj);
    void unvalidObjectCells(p3d_obj* obj);
  protected:
     void getCellListForEdge(p3d_polyhedre * poly, int edgeId, std::vector<DpgCell*>& edgeCells);
     void getCellForProjectedEdge(double * point1, double * point2, std::vector<DpgCell*>& edgeCells);
  private:
    p3d_env* _env;
    double _cellSize;
    //The position of the origin of the grid regarding th eorigin of the world
    double _originPos[3];
    int _nbMaxCells; //the number of cell along the longest axis of the environment
    int _nbCells;
    int _nbCellsX;
    int _nbCellsY;
    int _nbCellsZ;
    std::vector<DpgCell*> _cells;
};

#endif
