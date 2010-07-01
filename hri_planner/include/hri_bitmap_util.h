#ifndef HRI_BTMAP_UTIL
#define HRI_BTMAP_UTIL


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "math.h"

int on_map(int x, int y, int z, hri_bitmap* bitmap);

int get_direction(hri_bitmap_cell *satellite_cell, hri_bitmap_cell *center_cell);

int isHardEdge(hri_bitmap_cell *last_cell, hri_bitmap_cell *middle_cell);

hri_bitmap* hri_bt_get_bitmap(int type, hri_bitmapset* bitmapset);

hri_bitmap_cell* hri_bt_get_cell(hri_bitmap* bitmap, int x, int y, int z);

hri_bitmap_cell* hri_bt_get_closest_cell(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z);

hri_bitmap_cell* hri_bt_get_closest_free_cell(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z, double orientation, int max_grid_tolerance);

int hri_bt_isRobotOnCellInCollision(hri_bitmapset * btset, hri_bitmap* bitmap, hri_bitmap_cell* cell, double orientation, int checkHumanCollision);

hri_bitmap_cell* hri_bt_getCellOnPath(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z);

void hri_bt_copy_bitmap_values(hri_bitmap* bitmap_source, hri_bitmap* bitmap_target);

hri_bitmap* hri_bt_create_copy(hri_bitmap* bitmap);

hri_bitmapset*  hri_bt_create_empty_bitmapset();

hri_bitmap*  hri_bt_create_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int));

int hri_bt_create_data(hri_bitmap* bitmap);

hri_bitmap*  hri_bt_create_empty_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int));

int hri_bt_change_bitmap_position(hri_bitmapset * btset, double x, double y, double z);

int hri_bt_destroy_bitmap(hri_bitmap* bitmap);

int hri_bt_destroy_bitmap_data(hri_bitmap* bitmap);

int hri_bt_equalPath(hri_bitmap* bitmap1, hri_bitmap* bitmap2);

int localPathCollides (hri_bitmapset * btset, hri_bitmap_cell* cell, hri_bitmap_cell* fromcell);

double getCellDistance (hri_bitmap_cell* cell1, hri_bitmap_cell* cell2 );

double getAngleDeviation(double angle1, double angle2);

double getRotationBoundingCircleRadius(p3d_rob *robot);

#endif

