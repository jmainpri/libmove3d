#ifndef __DPGGRID_H__
#define __DPGGRID_H__
#include "P3d-pkg.h"
#include "../planner/dpg/proto/DpgCell.h"
#include <vector>
#include "../planner_cxx/API/3DGrid/grid.h"

class DpgGrid : public API::Grid{
  
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
    //1 = vers les positifs, -1 = vers les negatifs, 0 retourner toutes les cellules
    std::vector<DpgCell*> getCellPoint(double *point, int dx, int dy, int dz);
    void init(void);
    std::vector<DpgCell*> getCellListForObject(p3d_obj* obj);
    void unvalidObjectCells(p3d_obj* obj);
    void draw();
  protected:
     void getCellListForEdge(p3d_polyhedre * poly, int edgeId, std::vector<DpgCell*>& edgeCells);
     void getCellListForEdge(double * point1, double * point2, std::vector<DpgCell*>& edgeCells);
     int pointInPolygon(int nbPolyPoints, double* absys, double * ordon, double pointAbsys, double pointOrdon);
     void getCellListForFace(p3d_polyhedre * poly, int faceId, std::vector<DpgCell*>& edgeCells);
     DpgCell* createNewCell(int index, int x, int y, int z );
  private:
    p3d_env* _env;
    //The position of the origin of the grid regarding th eorigin of the world
    int _nbMaxCells; //the number of cell along the longest axis of the environment
    int _nbCells;
};

#endif
