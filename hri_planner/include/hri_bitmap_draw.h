#ifndef HRI_BTMAP_DRAW
#define HRI_BTMAP_DRAW


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"

int hri_bt_insert_obs(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_obj* obj, p3d_env* env, double expand, double value, int manip);
int hri_bt_insert_obsrobot(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_rob* obj, p3d_env* env, double expand, double value, int manip);
int  hri_bt_fill_bitmap_zone(hri_bitmapset * btset, hri_bitmap* bitmap, double xmin, double xmax, double ymin,
    double ymax, double zmin, double zmax, double expand, int val, int manip);
void hri_bt_show_path(hri_bitmapset * btset, hri_bitmap* bitmap);
void hri_bt_clearCorridorMarks(hri_bitmapset * btset, hri_bitmap* bitmap);

#endif

