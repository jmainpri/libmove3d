/*
 * hri_bitmap_bint_heap.h
 *
 *  Created on: Jun 18, 2009
 *      Author: kruset
 */

#ifndef HRI_BITMAP_BINT_HEAP_H_
#define HRI_BITMAP_BINT_HEAP_H_


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"



int hri_bt_A_Heap_size();
int hri_bt_A_insert_OL(hri_bitmap_cell *cell);
hri_bitmap_cell* hri_bt_A_remove_OL();
int hri_bt_A_update_cell_OL(hri_bitmap_cell *cell);

void hri_bt_init_BinaryHeap(hri_bitmap *cell);
void hri_bt_destroy_BinaryHeap();

#endif /* HRI_BITMAP_BINT_HEAP_H_ */
