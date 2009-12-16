#include "../planner/dpg/proto/DpgGrid.h"

using namespace std;

DpgGrid::DpgGrid(p3d_env * env){
  _env = env;
  _nbMaxCells = 30;
  _cellSize = env->box.x2 - env->box.x1;
  _cellSize = MAX(env->box.y2 - env->box.y1, _cellSize);
  _cellSize = MAX(env->box.z2 - env->box.z1, _cellSize);
  _cellSize /= _nbMaxCells;

  _originPos[0] = env->box.x1;
  _originPos[1] = env->box.y1;
  _originPos[2] = env->box.z1;

  _nbCellsX = (env->box.x2 - env->box.x1)/_cellSize;
  _nbCellsY = (env->box.y2 - env->box.y1)/_cellSize;
  _nbCellsZ = (env->box.z2 - env->box.z1)/_cellSize;

  _nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;

  for(int i = 0; i < _nbCells; i++){
    _cells.push_back(new DpgCell(i));
  }

  //Build the meshes env edges
  if(env->o[0]->pol[0]->poly->the_edges == NULL){
    for(int i = 0; i < env->no; i++){
      p3d_obj * obj = env->o[i];
      for(int j = 0; j < obj->np; j++){
        poly_build_edges(obj->pol[j]->poly);
      }
    }
  }
}

DpgGrid::~DpgGrid(){
  for(int i = 0; i < _nbCells; i++){
    if(_cells[i] != NULL){
      delete(_cells[i]);
      _cells[i] =  NULL;
    }
  }
}

void DpgGrid::init(void){
  for(int i = 0; i < _env->no; i++){
    unvalidObjectCells(_env->o[i]);
  }
}

vector<DpgCell*> DpgGrid::getCellListForObject(p3d_obj* obj){
  vector<DpgCell*> edgeCells, objectCells;
  vector<DpgCell*>::iterator it;

  for(int i = 0; i < obj->np; i++){
    if(obj->pol[i]->TYPE != P3D_GRAPHIC){
      p3d_polyhedre * poly = obj->pol[i]->poly;
      for(unsigned int j = 0; j < poly->nb_edges; j++){
        getCellListForEdge(poly, j, edgeCells);
        //Add edge cell to object cells
        objectCells.insert(objectCells.end(), edgeCells.begin(), edgeCells.end());
      }
    }
  }
  return objectCells;
}

void DpgGrid::unvalidObjectCells(p3d_obj* obj){
  vector<DpgCell*> cellTab = getCellListForObject(obj);
  for(unsigned int i = 0; i < cellTab.size(); i++){
    cellTab[i]->unvalid();
  }
}

vector<DpgCell*> DpgGrid::getCellPoint(double *point, int dx = 0, int dy = 0, int dz = 0){
  vector<DpgCell*> pointCells;
  int x = 0, y = 0, z = 0;
  switch(dx){
    case -1:{
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      break;
    }
    case 1:{
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      break;
    }
  }
  switch(dy){
    case -1:{
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      break;
    }
    case 1:{
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      break;
    }
  }
  switch(dz){
    case -1:{
      z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
      break;
    }
    case 1:{
      z = (int)floor(point[2]-_originPos[2]/_cellSize);
      break;
    }
  }
  if(dx == 0){//on a YZ plan
    if(dy == 0){//4 cells
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else if(dz == 0){//4cells
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      z = (int)floor(point[2]-_originPos[2]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      z = (int)floor(point[2]-_originPos[2]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else{//2cells
      x = (int)floor(point[0]-_originPos[0]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor(point[0]-_originPos[0]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }
  }else if(dy == 0){//on a XZ plan
    if(dz == 0){//4cells
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      z = (int)floor(point[2]-_originPos[2]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      z = (int)floor(point[2]-_originPos[2]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else{//2cells
      y = (int)floor(point[1]-_originPos[1]/_cellSize) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor(point[1]-_originPos[1]/_cellSize);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }
  }else if(dz == 0){//on a XY plan
    z = (int)floor(point[2]-_originPos[2]/_cellSize) - 1;
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    z = (int)floor(point[2]-_originPos[2]/_cellSize);
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
  }else{//1cell
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
  }
  return pointCells;
}

//protected functions

void DpgGrid::getCellListForEdge(p3d_polyhedre * poly, int edgeId, vector<DpgCell*>& edgeCells){
  double point1[2] = {0, 0}, z1 = 0;
  double point2[2] = {0, 0}, z2 = 0;
  //Recuperer les coordonnees des deux extremitees de l'arrete
  poly_get_point_in_edge(poly, edgeId, 1, &point1[0], &point1[1], &z1);
  poly_get_point_in_edge(poly, edgeId, 2, &point2[0], &point2[1], &z2);

  //projection sur XY
  //taille maximale du nombre de cellules
//   nb2dCells = (ABS(point2[0]/grid->cellSize - point1[0]/grid->cellSize) + 1) *  (ABS(point2[1]/grid->cellSize - point1[1]/grid->cellSize) + 1);
  getCellForProjectedEdge(point1, point2, edgeCells);

  //projection sur YZ
  //         point1[0] = point1[1];
  //         point1[1] = z1;
  //         point2[0] = point2[1];
  //         point2[1] = z2;
  //         edgeDir[0] = point2[1] - point1[1];
  //         edgeDir[1] = point2[2] - point1[2];
}

void DpgGrid::getCellForProjectedEdge(double * point1, double * point2, vector<DpgCell*>& edgeCells){
  //Cells for start point
  double p1p2[3] = {point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]};
  int dx = ABS(p1p2[0]) < EPS6 ? 0 : p1p2[0] > 0 ? 1 : -1;
  int dy = ABS(p1p2[1]) < EPS6 ? 0 : p1p2[1] > 0 ? 1 : -1;
  int dz = ABS(p1p2[2]) < EPS6 ? 0 : p1p2[2] > 0 ? 1 : -1;
  vector<DpgCell*> pointCells = getCellPoint(point1, dx, dy, dz);
  edgeCells.insert(edgeCells.end(), pointCells.begin(), pointCells.end());
  //Cells for end point
  pointCells.clear();
  pointCells = getCellPoint(point1, -dx, -dy, -dz);
  edgeCells.insert(edgeCells.end(), pointCells.begin(), pointCells.end());

  
//   //Check the rest of the edge
//   //start with the projection over XY plan
//   if(dx < 0){
//     //First determine the equation of the line (P2P1) y = ax + b
//     double a = (-p1p2[1]) / (-p1p2[0]);
//     double b = point2[1] - a * point2[0];
//     //get the intersection with all the cells border parallel to X than parallel to Y
//     startAxis = (int)floor(point2[0]-_originPos[0]/_cellSize) + 1;
//     stopAxis = (int)floor(point1[0]-_originPos[0]/_cellSize);
//     for(int i = startAxis; i < stopAxis; i++){
//       double p[3];
//       p[0] = i * _cellSize;
//       p[1] = a * x + b;
//       p[2] = 0;
//     }
//   }else if(dx > 0){
//     //First determine the equation of the line (P1P2) y = ax + b
//     double a = (p1p2[1]) / (p1p2[0]);
//     double b = point1[1] - a * point1[0];
//     //get the intersection with all the cells border parallel to X than parallel to Y
//     
//   }else{//dx == 0
// 
//   }
}
