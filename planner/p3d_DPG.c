// #ifdef DPG
#include "Planner-pkg.h"
#include "P3d-pkg.h"
//the number of cell along the longest axis of the environment
#define NBCELL 30

static p3d_dpgCell ** p3d_getCellListForObject(p3d_obj* obj, p3d_dpgGrid * grid, int* nbCells);
static int line_line_intersection2D(double point1[2], double direction1[2], double point2[2], double direction2[2], double result[2]);
static int p3d_mat2Invert(p3d_matrix2 mat, p3d_matrix2 invmat);


p3d_dpgGrid * p3d_allocDPGGrid(){
  p3d_dpgGrid * grid = NULL;

  grid = MY_ALLOC(p3d_dpgGrid, 1);
  return grid;
}

void p3d_destroyDPGGrid(p3d_dpgGrid ** grid){
  for(int i = 0; i < (*grid)->nbCells; i++){
    if((*grid)->cells != NULL){
      p3d_destroyDPGCell(&((*grid)->cells[i]));
    }
  }
  MY_FREE((*grid)->cells, p3d_dpgCell*, (*grid)->nbCells);
  MY_FREE(*grid, p3d_dpgGrid, 1);
}

void p3d_initDPGGrid(p3d_env * env, p3d_dpgGrid * grid){
  grid->cellSize = env->box.x2 - env->box.x1;
  grid->cellSize = MAX(env->box.y2 - env->box.y1, grid->cellSize);
  grid->cellSize = MAX(env->box.z2 - env->box.z1, grid->cellSize);
  grid->cellSize /= NBCELL;

  grid->originPos[0] = env->box.x1;
  grid->originPos[1] = env->box.y1;
  grid->originPos[2] = env->box.z1;

  grid->nbCellsX = (env->box.x2 - env->box.x1)/grid->cellSize;
  grid->nbCellsY = (env->box.y2 - env->box.y1)/grid->cellSize;
  grid->nbCellsZ = (env->box.z2 - env->box.z1)/grid->cellSize;

  grid->nbCells = grid->nbCellsX * grid->nbCellsY * grid->nbCellsZ;

  grid->cells = MY_ALLOC(p3d_dpgCell *, grid->nbCells);
  for(int i = 0; i < grid->nbCells; i++){
    grid->cells[i] = p3d_allocDPGCell();
    p3d_initDPGCell(grid->cells[i]);
    grid->cells[i]->id = i;
  }
}

p3d_dpgCell * p3d_allocDPGCell(){
  p3d_dpgCell * cell = NULL;

  cell = MY_ALLOC(p3d_dpgCell, 1);
  return cell;
}

void p3d_destroyDPGCell(p3d_dpgCell ** cell){
  MY_FREE(*cell, p3d_dpgCell, 1);
  *cell = NULL;
}

void p3d_initDPGCell(p3d_dpgCell * cell){
  cell->id = -1;
  cell->nbEdges = 0;
  cell->edges = NULL;
  cell->valid = -1;
}

void p3d_initStaticGrid(p3d_env * env, p3d_dpgGrid * grid){
  p3d_dpgCell ** cellTab = NULL;
  int nbCells = 0;
  for(int i = 0; i < env->no; i++){
    cellTab = p3d_getCellListForObject(env->o[i], grid, &nbCells);
    for(int j = 0; j < nbCells; j++){
      cellTab[j]->valid = 0;
    }
    if(cellTab != NULL){
      MY_FREE(cellTab, p3d_dpgCell *, nbCells);
      cellTab = NULL;
      nbCells = 0;
    }
  }
}

static int ** p3d_get2dCellOnAxis(){

}

static void p3d_getCellForProjectedEdge(double * point1, double * point2, p3d_dpgGrid * grid, int** cells2d){
  double point3[2] = {0, 0};
  double result[2] = {0, 0};
  double edgeDir[2] = {0, 0};
  double axisDir[2] = {0, 0};
  int startAxis = 0, stopAxis = 0;

  edgeDir[0] = point2[0] - point1[0];
  edgeDir[1] = point2[1] - point1[1];
  if(point1[1] == point2[1]){
    printf("Yaxis\n");
    point3[1] = 0;
    axisDir[0] = 0;
    axisDir[1] = 1;
    if(edgeDir[0] > 0){
      startAxis = point1[0]/grid->cellSize + 1;
      stopAxis = point2[0]/grid->cellSize;
    }else{
      startAxis = point2[0]/grid->cellSize + 1;
      stopAxis = point1[0]/grid->cellSize;
    }
    //TODO bornes de l'environnement
    for(int k = startAxis; k < stopAxis; k++){
      point3[0] = k * grid->cellSize;
      if(line_line_intersection2D(point1, edgeDir, point3, axisDir, result)){
        if(edgeDir[1] * point1[1] > edgeDir[1] * result[1] || edgeDir[1] * point2[1] < edgeDir[1] * result[1]){
          printf("outOfbounds\n");
        }else{
          printf("axe = %f, resultX = %f, resultY = %f\n", point3[0], result[0], result[1]);
        }
      }
    }
  }else{
    printf("Xaxis\n");
    point3[0] = 0;
    axisDir[0] = 1;
    axisDir[1] = 0;
    if(edgeDir[1] > 0){
      startAxis = point1[1]/grid->cellSize + 1 ;
      stopAxis = point2[1]/grid->cellSize;
    }else{
      startAxis = point2[1]/grid->cellSize + 1;
      stopAxis = point1[1]/grid->cellSize;
    }
    //lors d'une intersection on prends les deux cases audessus et en dessous de l'intersection, oua droite et gauche. Si on se trouve au croisement, on recupere les  cases autour.

    for(int k = startAxis, h = 0; k < stopAxis; k++, h += 4){
      point3[1] = k * grid->cellSize;
      if(line_line_intersection2D(point1, edgeDir, point3, axisDir, result)){
        if(edgeDir[0] * point1[0] > edgeDir[0] * result[0] || edgeDir[0] * point2[0] < edgeDir[0] * result[0]){
          printf("outOfbounds\n");
        }else{
          //regarder si c'est a l'intersection avec un axe parallele a Y
          if(ABS(result[0]/grid->cellSize) - floor(ABS(result[0]/grid->cellSize)) < EPS6){
            cells2d[h] = MY_ALLOC(int, 2);
            cells2d[h+1] = MY_ALLOC(int, 2);
            cells2d[h+2] = MY_ALLOC(int, 2);
            cells2d[h+3] = MY_ALLOC(int, 2);
            cells2d[h][0] = cells2d[h+1][0] = (result[0] - grid->originPos[0]) / grid->cellSize;
            cells2d[h+2][0] = cells2d[h+3][0] = (result[0] - grid->originPos[0]) / grid->cellSize - 1;
            cells2d[h][1] = cells2d[h+2][1] = k - grid->originPos[1] / grid->cellSize;
            cells2d[h+1][1] = cells2d[h+3][1] = k - grid->originPos[1] / grid->cellSize - 1;
          }else{
            cells2d[h] = MY_ALLOC(int, 2);
            cells2d[h+1] = MY_ALLOC(int, 2);
            cells2d[h][0] = cells2d[h+1][0] = (result[0] - grid->originPos[0]) / grid->cellSize;
            cells2d[h][1] = k - grid->originPos[1] / grid->cellSize;
            cells2d[h+1][1] = k - grid->originPos[1] / grid->cellSize - 1;
          }
          printf("axe = %f, resultX = %f, resultY = %f\n",point3[1], result[0], result[1]);
        }
      }
    }
  }
}

static p3d_dpgCell ** p3d_getCellListForEdge(p3d_polyhedre * poly, int edgeId, p3d_dpgGrid * grid, int* nbCells){
  double point1[2] = {0, 0}, z1 = 0;
  double point2[2] = {0, 0}, z2 = 0;
  int nb2dCells = 0;
  int ** cell2d = NULL;
  //Recuperer les coordonnees des deux extremitees de l'arrete
  poly_get_point_in_edge(poly, edgeId, 1, &point1[0], &point1[1], &z1);
  poly_get_point_in_edge(poly, edgeId, 2, &point2[0], &point2[1], &z2);

  //projection sur XY
  //taille maximale du nombre de cellules
  nb2dCells = (ABS(point2[0]/grid->cellSize - point1[0]/grid->cellSize) + 1) *  (ABS(point2[1]/grid->cellSize - point1[1]/grid->cellSize) + 1);
  cell2d = MY_ALLOC(int *, nb2dCells);
  for(int i = 0; i < nb2dCells; i++){
    cell2d[i] = NULL;
  }
  p3d_getCellForProjectedEdge(point1,point2, grid, cell2d);

  //projection sur YZ
  //         point1[0] = point1[1];
  //         point1[1] = z1;
  //         point2[0] = point2[1];
  //         point2[1] = z2;
  //         edgeDir[0] = point2[1] - point1[1];
  //         edgeDir[1] = point2[2] - point1[2];
}

static p3d_dpgCell ** p3d_getCellListForObject(p3d_obj* obj, p3d_dpgGrid * grid, int* nbCells){
  int polyNbCells = 0;
  for(int i = 0; i < obj->np; i++){
    if(obj->pol[i]->TYPE != P3D_GRAPHIC){
      p3d_polyhedre * poly = obj->pol[i]->poly;
      for(int j = 0; j < poly->nb_edges; j++){
        p3d_getCellListForEdge(poly, j, grid, &polyNbCells);
      }
    }
  }
  return NULL;
}

// Cette fonction calcule l'intersection entre deux droites dans le plan.
// Ces droites sont caractérisées par un de leur point et leur direction.
// Retourne le nombre d'intersections (0 ou 1).
static int line_line_intersection2D(double point1[2], double direction1[2], double point2[2], double direction2[2], double result[2])
{
  double lambda1;
  p3d_matrix2 M, invM;
  M[0][0]=   direction1[0];     M[0][1]=   -direction2[0];
  M[1][0]=   direction1[1];     M[1][1]=   -direction2[1];

  if( p3d_mat2Invert(M, invM)==0 )
    return 0;
  else
  {
     lambda1= invM[0][0]*(point2[0] - point1[0]) + invM[0][1]*(point2[1] - point1[1]);

     result[0]= point1[0] + lambda1*direction1[0];
     result[1]= point1[1] + lambda1*direction1[1];

     return 1;
  }
}

// Calcule l'inverse d'une matrice 2x2.
//EPSILON doit être défini par ailleurs
static int p3d_mat2Invert(p3d_matrix2 mat, p3d_matrix2 invmat)
{
  double det= mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1];
  if( fabs(det) < EPS6)
  {
     #ifdef DEBUG
       printf("%s: %d: p3d_mat2Invert(): matrice non inversible\n", __FILE__, __LINE__);
     #endif
     return 0;
  }

  invmat[0][0]=   mat[1][1]/det;    invmat[0][1]=  -mat[0][1]/det;
  invmat[1][0]=  -mat[1][0]/det;    invmat[1][1]=   mat[0][0]/det;

  return 1;
}

void buildEnvEdges(p3d_env * env){
  for(int i = 0; i < env->no; i++){
    p3d_obj * obj = env->o[i];
    for(int j = 0; j < obj->np; j++){
      poly_build_edges(obj->pol[j]->poly);
    }
  }
}

static p3d_dpgCell * p3d_getCell(p3d_dpgGrid * grid, int x, int y, int z){
  return grid->cells[x + y * (grid->nbCellsX) + z * (grid->nbCellsX * grid->nbCellsY)];
}
// #endif
