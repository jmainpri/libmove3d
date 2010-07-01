#include "DpgGrid.h"
#include "ThreeDCell.h"

using namespace std;

DpgGrid::DpgGrid(p3d_env * env){
  _env = env;
  _nbMaxCells = 30;
  double cellSize = env->box.x2 - env->box.x1;
  cellSize = MAX(env->box.y2 - env->box.y1, cellSize);
  cellSize = MAX(env->box.z2 - env->box.z1, cellSize);
  cellSize /= _nbMaxCells;

  _originCorner[0] = env->box.x1;
  _originCorner[1] = env->box.y1;
  _originCorner[2] = env->box.z1;

  _nbCellsX = (env->box.x2 - env->box.x1)/cellSize;
  _nbCellsY = (env->box.y2 - env->box.y1)/cellSize;
  _nbCellsZ = (env->box.z2 - env->box.z1)/cellSize;

  _nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
  _cellSize[0]= _cellSize[1] = _cellSize[2] = cellSize;
  this->createAllCells();

  //Build the meshes env edges
  if(env->o[0]->pol[0]->poly->the_edges == NULL){
    for(int i = 0; i < env->no; i++){
      p3d_obj * obj = env->o[i];
      for(int j = 0; j < obj->np; j++){
        poly_build_edges(obj->pol[j]->poly);
      }
    }
  }
  init();
}

DpgGrid::~DpgGrid(){
  for(int i = 0; i < _nbCells; i++){
    if(static_cast<DpgCell*>(BaseGrid::getCell(i)) != NULL){
      delete(static_cast<DpgCell*>(BaseGrid::getCell(i)));
    }
  }
}

void DpgGrid::init(void){
  p3d_compute_static_objects_point_cloud(_env, _cellSize[0]*0.25);
  p3d_compute_all_robots_bodies_point_cloud(_env, _cellSize[0]*0.25);
  //unvalid static object Cells
  for(int i = 0; i < _env->no; i++){
    unvalidObjectCells(_env->o[i]);
  }
  //unvalid cells for each robot except current one ?
}

void DpgGrid::updateRobotOccupationCells(p3d_rob* robot){
  vector<DpgCell*> robotCell;
  for(int i = 0; i <= robot->njoints; i++){
    if(robot->joints[i]->o){
      vector<DpgCell*> objectCell = getCellListForObject(robot->joints[i]->o, robot->joints[i]->abs_pos);
      robotCell.insert(robotCell.end(), objectCell.begin(), objectCell.end());
    }
  }
  MY_FREE(robot->dpgCells, DpgCell*,  robot->nbDpgCells);
  robot->nbDpgCells = robotCell.size();
  robot->dpgCells = MY_ALLOC(DpgCell* , robot->nbDpgCells);
  for(unsigned int i = 0; i < robotCell.size(); i++){
    robot->dpgCells[i] = robotCell[i];
  }
}

void DpgGrid::unvalidObjectCells(p3d_obj* obj){
  vector<DpgCell*> cellTab = getCellListForObject(obj, p3d_mat4IDENTITY);
  for(unsigned int i = 0; i < cellTab.size(); i++){
    cellTab[i]->setValid(false);
  }
}

vector<DpgCell*> DpgGrid::getCellListForObject(p3d_obj* obj, p3d_matrix4 pointTransform){
  vector<DpgCell*> objectCells;
  for(unsigned int i = 0; i < obj->nbPointCloud; i++){
    p3d_vector3 point;
    p3d_xformPoint(pointTransform, obj->pointCloud[i], point);
    DpgCell* cell = (DpgCell*)(getCell(point));
    if(!cell->isVisited()){
      cell->setVisited(true);
      objectCells.push_back(cell);
    }
  }
  for(unsigned int i = 0; i < objectCells.size(); i++){
    objectCells[i]->setVisited(false);
  }
  return objectCells;
}

void DpgGrid::draw(){
  for(unsigned int i=0; i < getNumberOfCells(); i++){
    DpgCell* cell = dynamic_cast<DpgCell*>(BaseGrid::getCell(i));
    if(!cell->isValid()){
      cell->draw();
    }
  }
  for(unsigned int i=0; i < getNumberOfCells(); i++){
    DpgCell* cell = dynamic_cast<DpgCell*>(BaseGrid::getCell(i));
    if(cell->isValid()){
      cell->draw();
    }
  }
}

//protected functions
/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param index the cell index
 * \param x The position of the cell over x
 * \param y The position of the cell over y
 * \param z The position of the cell over z
 */
API::ThreeDCell* DpgGrid::createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z ){
    DpgCell* newCell = new DpgCell( index, computeCellCorner(x,y,z) , this );
    return newCell;
}