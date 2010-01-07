/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern void compute_length_array ( pp3d_rob robotPt );
extern void *p3d_beg_desc ( int type, char *name );
extern int p3d_end_desc ( void );
extern int p3d_inside_desc ( void );
extern int p3d_add_desc_jnt_deg ( p3d_type_joint type, p3d_matrix4 pos, double * dtab, int prev, double * dtab2, double scale );
extern void p3d_add_desc_poly ( char name[20], int type );
extern void p3d_add_desc_vert ( double x, double y, double z );
extern void p3d_add_desc_face ( int *listeV, int nb_Vert );
extern void p3d_end_desc_poly ( void );
extern void p3d_add_desc_cube ( char name[20], double a, int type );
extern void p3d_add_desc_box ( char name[20], double a, double b, double c, int type );
extern void p3d_add_desc_srect ( char name[20], double a, double b, double h, double ux, double uy, double uz, double vx, double vy, double vz, int type );
extern void p3d_add_desc_pyramid ( char name[20], double a, double b, double c, double d, double e, double f, double g, int type );
extern void p3d_add_desc_cylindre ( char name[20], double r, double l, int type );
extern void p3d_add_desc_cylindre_oval ( char name[20], double a, double b, double l, int type );
extern void p3d_add_desc_prisme ( char name[20], int nvert, double r, double l, int type );
extern void p3d_add_desc_cone ( char name[20], int nvert, double r1, double r2, double l, int type );
extern void p3d_add_desc_snout ( char name[20], int nvert, double dt, double db, double height, double xoff, double yoff, int type );
extern void p3d_add_desc_skew_snout ( char name[20], int nvert, double dt, double db, double height, double xoff, double yoff, double a1, double a2, double a3, double a4, int type );
extern void p3d_add_desc_rtorusslice ( char name[20], double r1, double r2, double height, double angle, int type );
extern void p3d_add_desc_ctorusslice ( char name[20], double R, double radius, double angle, int type );
extern void p3d_add_desc_sphere ( char name[20], double r, int type );
extern void p3d_add_desc_half_sphere ( char name[20], double r, int type );
extern void p3d_add_desc_sphere_shell ( char name[20], double r, double h, int type );
extern void p3d_add_desc_sweptrectslice ( char name[20], double r1, double r2, double l, double a, double ux, double uy, double uz, double vx, double vy, double vz, int type );
extern void p3d_add_desc_oval ( char name[20], double a, double b, double c, int type );
extern void p3d_add_desc_half_oval ( char name[20], double a, double b, double c, int type );
extern void p3d_add_desc_tore ( char name[20], double r, double a, int type );
extern int p3d_add_desc_courbe ( p3d_localpath *localpathPt );
extern void p3d_set_obst_color ( char *name, int color, double *color_vect );
extern void p3d_set_obst_poly_color ( char *name, int num, int color, double *color_vect );
extern void p3d_set_body_color ( char *name, int color, double *color_vect );
extern void p3d_set_body_poly_color ( char *name, int num, int color, double *color_vect );
extern void *p3d_sel_desc_name ( int type, char* name );
extern void *p3d_sel_desc_num ( int type, int num );
extern void *p3d_sel_desc_id ( int type, void *id );
extern int p3d_get_number_joints_current_robot ( void );
extern int p3d_get_desc_number ( int type );
extern char *p3d_get_desc_curname ( int type );
extern void *p3d_get_desc_curid ( int type );
extern int p3d_get_desc_curnum ( int type );
extern void *p3d_beg_env ( char* name );
extern void *p3d_beg_obj ( char *name, int type );
extern int return_R_DEF ( void );
extern int trj_set_null ( int traj_id );
extern void move_point ( p3d_matrix4 pos, double *x, double *y, double *z, int point );
#ifdef MULTIGRAPH
extern void p3d_initMultiGraph(p3d_rob* robot, p3d_multiGraph* mg);
extern p3d_multiGraph* p3d_cloneMultiGraph(p3d_rob* robot, p3d_multiGraph* src);
extern int p3d_set_multi_graph_data(p3d_rob* r, int nbJoints, int *joints);
#endif
#ifdef LIGHT_PLANNER
extern int p3d_set_removable_bb_for_grasp(p3d_rob* r, int nbJoints, int *joints);
#endif
#ifdef MULTILOCALPATH
extern int p3d_set_multi_localpath_group(p3d_rob* r, int nbJoints, int *joints, int activated);
extern int p3d_set_multi_localpath_data(p3d_rob* r, const char* gp_name_in, const char* gp_type_in, char* lpl_type_in, double *dtab);
#endif

#endif /* __CEXTRACT__ */
