/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue May 27 15:32:07 2008
 */
#ifndef __CEXTRACT__

extern hri_bitmap* hri_bt_create_bitmap ( int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*, int, int, int) );
extern int hri_bt_create_data ( hri_bitmap* bitmap );
extern hri_bitmap* hri_bt_create_empty_bitmap ( int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*, int, int, int) );
extern int hri_bt_change_bitmap_position ( hri_bitmapset * btset, double x, double y, double z );
extern int hri_bt_destroy_bitmap ( hri_bitmap* bitmap );
extern int hri_bt_destroy_bitmap_data ( hri_bitmap* bitmap );
extern int hri_bt_destroy_bitmapset ( hri_bitmapset* bitmapset );
extern int hri_bt_destroy_human ( hri_human* human );
extern int hri_bt_destroy_state ( hri_human_state state );
extern int hri_bt_activate ( int type, hri_bitmapset* bitmapset );
extern int hri_bt_fill_bitmap ( hri_bitmapset * btset, int type );
extern int hri_bt_create_obstacles ( hri_bitmapset* btset );
extern int hri_bt_insert_obs ( hri_bitmapset * btset, hri_bitmap* bitmap, p3d_obj* obj, p3d_env* env, double expand, double value, int manip );
extern int hri_bt_fill_bitmap_zone ( hri_bitmap* bitmap, int objxmin, int objxmax, int objymin, int objymax, int objzmin, int objzmax, int val );
extern void hri_bt_show_path ( hri_bitmapset * btset, hri_bitmap* bitmap );
extern void hri_bt_show_cone ( hri_bitmapset * btset, hri_bitmap* bitmap, int h, int r );
extern void hri_bt_show_bitmap ( hri_bitmapset * btset, hri_bitmap* bitmap );
extern int hri_bt_save_bitmap ( hri_bitmapset* btset, hri_bitmap * B );
extern void hri_bt_show_bitmapset ( hri_bitmapset* bitmapset );
extern hri_bitmapset* hri_bt_create_bitmaps ( void );
extern int hri_bt_init_bitmaps ( hri_bitmapset * bitmapset, int x, int y, int z, double pace );
extern hri_human* hri_bt_create_human ( p3d_rob * robot );
extern int hri_bt_bitmap_to_GRAPH ( hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap );
extern void hri_bt_reset_path ( hri_bitmapset * btset );
extern hri_bitmap_cell* hri_bt_get_cell ( hri_bitmap* bitmap, int x, int y, int z );
extern double hri_bt_start_search ( double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip );
extern hri_bitmap* hri_bt_create_copy ( hri_bitmap* bitmap );
extern hri_bitmapset* hri_bt_create_empty_bitmapset ( void );
extern void g3d_hri_bt_draw_active_bitmaps ( hri_bitmapset* bitmapset );
extern void g3d_hri_bt_draw_targets ( hri_bitmapset* bitmapset );
extern void hri_bt_desactivate ( int type, hri_bitmapset* bitmapset );
extern int hri_bt_is_active ( int type, hri_bitmapset* bitmapset );
extern int hri_bt_refresh_all ( hri_bitmapset * btset );
extern int hri_bt_3drefresh_all ( hri_bitmapset * btset );
extern hri_bitmap* hri_bt_get_bitmap ( int type, hri_bitmapset* bitmapset );
extern void hri_bt_reset_bitmap_data ( hri_bitmap* B );
extern void hri_bt_init_TARGET ( int place, int type );
extern configPt hri_bt_set_TARGET ( void );
extern int hri_bt_close_cell ( hri_bitmap* bitmap, hri_bitmap_cell* current_cell );
extern double hri_bt_astar_bh ( hri_bitmapset * btset, hri_bitmap* bitmap );
extern int hri_bt_A_insert_OL ( hri_bitmap_cell *cell );
extern hri_bitmap_cell* hri_bt_A_remove_OL ( void );
extern int hri_bt_A_update_cell_OL ( hri_bitmap_cell *cell );
extern int hri_bt_A_neigh_costs ( hri_bitmapset* btset, hri_bitmap* bitmap, hri_bitmap_cell* center_cell, hri_bitmap_cell* final_cell);
extern int hri_bt_update_distance ( hri_bitmapset * btset, double height, double radius );
extern int hri_bt_update_visibility ( hri_bitmapset * btset, double height, double p2, double p3 );
extern int hri_bt_update_hidzones ( hri_bitmapset * btset, double radius );
extern int hri_bt_update_combined ( hri_bitmapset * btset );
extern int hri_bt_sit_stand ( p3d_rob* human );
extern int hri_bt_write_TRAJ ( hri_bitmapset * btset, p3d_jnt * joint );
extern int hri_bt_verify_path ( hri_bitmapset * btset );
extern int hri_bt_verify_pathold ( hri_bitmapset * btset );
extern int hri_bt_destroy_path ( hri_bitmapset* bitmapset );
extern void hri_bt_print_PATH ( hri_bitmapset* bitmapset );
extern int hri_bt_opt_path ( hri_bitmap* bitmap );
extern int hri_bt_write_path2file ( void );
extern int hri_bt_calculate_bitmap_path ( hri_bitmapset * btset, p3d_rob *robotPt, configPt qs, configPt qg, int manip );
extern p3d_graph * hri_bt_create_graph ( p3d_rob* Robot );
extern void g3d_hri_bt_draw_active_3dbitmaps ( hri_bitmapset* bitmapset );
extern void hri_bt_show_3dbitmap ( hri_bitmapset* btset, hri_bitmap* bitmap );
extern double hri_bt_min_cell_limited ( hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double xlimit, double ylimit, double zlimit );
extern double hri_bt_min_cell ( hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z );
extern double hri_bt_max_cell ( hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z );
extern int hri_bt_calculate_bitmap_pathwGIK ( hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip );
extern int hri_bt_bitmap_to_graphwGIK ( hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap );
extern void hri_bt_min_cell_n ( hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double * cost, int n );
extern int hri_change_human_state ( hri_human * human, int state, configPt config );
extern int hri_bt_create_precise_obstacles ( hri_bitmapset * bitmapset );
extern int hri_bt_calculate_bitmap_pathwR6IK ( hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip );
extern int hri_bt_bitmap_to_graphwR6IK ( hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap );
extern gnuplot_ctrl * hri_bt_init_gnuplot ( double xmin, double xmax, double ymin, double ymax, double zmin, double zmax );
extern gnuplot_ctrl * hri_bt_init_gnuplot_bitmap(hri_bitmapset * btset, int btno);
extern int hri_bt_gnuplot_bitmap ( gnuplot_ctrl * h, hri_bitmapset * btset, int btno, double exclude );
extern hri_bitmapset* hri_bt_create_bitmapsworobots( void );
extern int hri_set_human_state(hri_human * human,int state, configPt  config );
extern int hri_set_human_state_SICK(hri_human * human, int state, configPt config, int adjustForSick );
extern int hri_bt_insert_obsrobot(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_rob* obj, p3d_env* env, double expand, double value, int manip);
#endif /* __CEXTRACT__ */
