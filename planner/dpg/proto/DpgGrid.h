#ifndef __DPGGRID_H__
#define __DPGGRID_H__
#include "P3d-pkg.h"
#include "DpgCell.h"
#include <vector>
#include "ThreeDGrid.h"

class DpgGrid : public Move3D::ThreeDGrid{
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
    void init(void);
    void updateRobotOccupationCells(p3d_rob* robot);
    std::vector<DpgCell*> getCellListForObject(p3d_obj* obj, p3d_matrix4 pointTransform);
    void unvalidObjectCells(p3d_obj* obj);
    void draw();
  protected:
  virtual Move3D::ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
  private:
    p3d_env* _env;
    //The position of the origin of the grid regarding th eorigin of the world
    int _nbMaxCells; //the number of cell along the longest axis of the environment
    int _nbCells;
};

#endif
