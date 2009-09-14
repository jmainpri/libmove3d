/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue Mar 17 18:04:54 2009
 */
#ifndef __CEXTRACT__

extern int psp_test_destination_to_robot ( p3d_rob *r, p3d_rob *objRob, hri_bitmapset* PSP_BTSET );
extern int psp_test_actual_robot_pos ( p3d_rob *r, p3d_rob *objRob, hri_bitmapset* PSP_BTSET );
extern int psp_srch_model_pt ( p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET );
extern int psp_srch_model_pt_obj ( p3d_rob* r, p3d_obj* object, int numsegs, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET );
extern int psp_srch_3D_model_pt_obj ( p3d_rob* r, p3d_obj* object, int numpoints, int OnSurface, hri_bitmapset* PSP_BTSET );
extern int psp_goto_look_obj ( p3d_rob* r, p3d_obj* object, int numpoints1, int numpoints2, int OnSurface, hri_bitmapset* PSP_BTSET );
extern int psp_srch_model_pt_searchball(psp_searchball *sball, p3d_rob* r,  int numsegs, int numlayers,  int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET);
extern int psp_srch_rnd_model_pt ( p3d_rob* r, p3d_rob* objRob, int numpoints, int numlayers, int *search_method, double viewPercent, hri_bitmapset* PSP_BTSET );
extern void psr_get_obj_list ( p3d_rob *currRob, p3d_obj **oList, int *nObj, p3d_rob **rList, int *nRob, double viewPercent );
extern void psr_get_obj_list_multi(p3d_rob *currRob, p3d_obj **oList, int nObj, p3d_obj **oListOut, int *nObjOut, double viewPercent);
extern void psr_get_rob_parts_list ( p3d_rob *currRob, p3d_obj **partsList, int *nParts, double viewPercent );
extern void psr_get_joint_attention ( hri_bitmapset* PSP_BTSET, double viewPercent );
extern void psr_get_human_left_pointing ( p3d_rob* human, p3d_rob* r, hri_bitmapset* PSP_BTSET );
extern void psr_get_human_pointing_from_joint_number ( p3d_rob* human, p3d_rob* r, int jntIdx, hri_bitmapset* PSP_BTSET );
extern void psr_get_pointing_from_joint ( p3d_rob* r, p3d_jnt *jntPt, int frameType, hri_bitmapset* PSP_BTSET );
extern void initpspGiks ( void );
extern int p3d_init_robot_parameters ( void );
extern int p3d_init_object_parameters_by_name ( char *objName, double min, double max );
extern int p3d_init_all_object_parameters_by_type ( char *objType, double min, double max );
extern void psp_search_for_objectives ( p3d_rob *robot, p3d_vector3 point );
extern int psp_srch_for_target_obj ( p3d_rob *robot, int numsegs, int numlayers, int searchMode, int *searchMtd, double viewpercent, hri_bitmapset* PSP_BTSET );
extern void psp_add_element ( psp_lst_elements *lstel, psp_obs_element *elem );
extern void p3d_select_robot_to_view ( p3d_rob *robotPt );
extern void p3d_deselect_robot_to_view ( p3d_rob *robotPt );
extern int p3d_get_rob_select_status ( p3d_rob *robotPt );
extern void p3d_deselect_all_objects ( void );
extern void p3d_select_object_to_view ( p3d_obj *objectPt );
extern int psp_select_object_to_view_by_name ( char *objName );
extern void p3d_unselect_object_to_view ( p3d_obj *objectPt );
extern int p3d_get_obj_select_status ( p3d_obj *objectPt );
extern void p3d_set_body_selection ( p3d_rob *r, int body, int val );
extern void g3d_psp_draw_lookatpoint ( p3d_rob *robot );
extern void p3d_get_robot_center ( p3d_rob* rob, p3d_vector4 pointc );
extern void p3d_get_object_center ( p3d_obj* obj, p3d_vector4 pointc );
extern double p3d_psp_pointtolinedist ( p3d_vector3 p, p3d_vector3 l1, p3d_vector3 l2 );
extern void p3d_psp_cartesian2spherical ( double x, double y, double z, double originx, double originy, double originz, double *phi, double *theta );
extern void p3d_psp_spherical2cartesian ( double x, double y, double z, double rad, double phi, double theta, p3d_vector4 point );
extern double angleLim ( double angle );
extern int p3d_psp_is_point_in_a_cone ( p3d_vector4 p, p3d_vector4 conep, p3d_vector4 conep2, double coneangle, double *distf );
extern void psp_draw_test ( void );
extern void psp_draw_search_ball ( psp_searchball *srchballpt );
extern void psp_draw_in_perspwin ( void );
extern int psp_init_bitmap_grids ( void );
extern void printListVtx ( psp_lst_vertex *lstVtx );
extern void printQcosts ( int *indexes, float *qcst, int numqs, psp_lst_vertex *lstVtx );
extern void psp_chng_show_st ( void );
extern void p3d_psp_set_search_ball_pos( double x, double y, double z);
extern void psp_deselect_all_robots();
extern void psp_deselect_all();
extern int psp_is_a_human(p3d_rob *r); 
extern double pso_watch3_obj();
extern int psp_is_object_visible(p3d_rob * robot, p3d_rob * object, double threshold);
extern int psp_seen_objects(p3d_rob* robot,  p3d_rob** list_of_seen_objects, double threshold);
extern int psp_set_device_pos_by_name(char *devName, double x, double y, double z, double th);

#endif /* __CEXTRACT__ */
