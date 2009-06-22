#include "hri_bitmap_bin_heap.h"

/*
 * hri_bitmap_bin_heap.c
 *
 *  this uses a binary tree where OPENLIST[x/2] is the parent of OPENLIST[x] for x>1.
 *  The tree grants that the root is always the cellwith the smalles costs, and
 *  that any parent has smaller or equal costs than its children.
 *
 *  Currently there can only be one such heap in use in the system (but we need just one for now).
 *
 *  Created on: Jun 18, 2009
 */

#define LEFTCHILD(x) OPENLIST[2*(x)]
#define RIGHTCHILD(x) OPENLIST[2*(x)+1]
#define PARENT(x) OPENLIST[(x)/2]

hri_bitmap_cell ** OPENLIST; // a binary heap of all opened cells
int maxsize = -1;
int OL_cellnbr; // number of elements in the heap

/*********************ASTAR**************************************/
/*!
 * \brief A* Gives the search cost of a cell
 *
 * \param cell the cell
 *
 * \return the cost
 */
/****************************************************************/
static inline double Cellcost(hri_bitmap_cell* cell)
{
  return (cell->h + cell->g);
}

void hri_bt_init_BinaryHeap(hri_bitmap *bitmap)
{
  if (maxsize > 0) {
    PrintError(("Memory Leak: Trying to recreate Binary Heap without destoying first."));
    hri_bt_destroy_BinaryHeap();
  }
  maxsize = bitmap->nx * bitmap->ny * bitmap->nz;
  OPENLIST = MY_ALLOC(hri_bitmap_cell*,bitmap->nx * bitmap->ny * bitmap->nz); /* first element of this array is not used */
  OL_cellnbr = 0;
}

void hri_bt_destroy_BinaryHeap()
{
  if (maxsize > 0) {
    MY_FREE(OPENLIST, hri_bitmap_cell*, maxsize);
  }
  maxsize = -1;
}

int hri_bt_A_Heap_size()
{
  return OL_cellnbr;
}

/***************************ASTAR********************************/
/*!
 * \brief A* insert to the OPENLIST, using binary heaps
 *
 * \param cell  cell to insert
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_A_insert_OL(hri_bitmap_cell *cell)
{
  int index;
  hri_bitmap_cell* temp;

  if(OL_cellnbr == 0){
    OPENLIST[1] = cell;
    OL_cellnbr++;
    return TRUE;
  }

  // add new cell to end
  OPENLIST[OL_cellnbr+1] = cell;
  OL_cellnbr++;
  index = OL_cellnbr;

  // rebalance the tree
  while(index != 1){
    if( Cellcost(OPENLIST[index]) < Cellcost(PARENT(index) ) ){
      temp = PARENT(index);
      PARENT(index) = OPENLIST[index];
      OPENLIST[index] = temp;
      index = index/2;
    }
    else
      break;
  }
  return TRUE;
}

/***************************ASTAR********************************/
/*!
 * \brief A* remove from the OPENLIST, using binary heaps
 *
 *
 * \return gives the removed cell or NULL for an empty list
 */
/****************************************************************/
hri_bitmap_cell* hri_bt_A_remove_OL()
{
  int parent, candidate;
  hri_bitmap_cell* temp, * result;

  if(OL_cellnbr == 0)
    return NULL;

  result = OPENLIST[1]; // root will be returned

  // replace root with last element
  OPENLIST[1] = OPENLIST[OL_cellnbr];
  OPENLIST[OL_cellnbr] = NULL;
  OL_cellnbr--;
  candidate = 1;

  while(TRUE){
    parent = candidate; // recursively try to replace candidate with one of its children
    if( 2*parent <= OL_cellnbr ){ // if left child exists
      // choose left child if it has lower costs
      if( Cellcost(OPENLIST[candidate]) >= Cellcost(LEFTCHILD(parent) ) ) candidate = 2*parent;
      if( 2 * parent + 1 <= OL_cellnbr ) { // if right child exists as well
        if( Cellcost(OPENLIST[candidate]) >= Cellcost(RIGHTCHILD(parent)) ) candidate = 2*parent+1;
      }
    }

    if(parent!=candidate){
      temp = OPENLIST[parent];
      OPENLIST[parent] = OPENLIST[candidate];
      OPENLIST[candidate] = temp;
    }
    else
      break;
  }
  return result;

}

/***************************ASTAR********************************/
/*!
 * \brief A* Update the OPENLIST, using binary heaps
 *
 * \param cell  cell to update
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_A_update_cell_OL(hri_bitmap_cell *cell)
{
  int i;
  int index = 0;
  hri_bitmap_cell *temp;

  if(!cell->open){
    PrintWarning(("A*:Accessing a nonexistent cell in OL"));
    return FALSE;
  }


//  find index i of cell in array
  for(i=1; i < OL_cellnbr+1; i++){
    if(OPENLIST[i] == cell) {
      index = i;
      break;
    }

  }

  if(i==OL_cellnbr+1 || index < 1)
    return FALSE;

  // bubble up cell in tree until it is first or it is more expensive than the parent cell
  while(index != 1) {
    if( Cellcost(OPENLIST[index]) < Cellcost(PARENT(index)) ){
      temp = OPENLIST[index];
      OPENLIST[index] = PARENT(index);
      PARENT(index) = temp;
      index = index/2;
    }
    else
      break;
  }
  return TRUE;

}


