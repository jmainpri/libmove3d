#ifndef __CEXTRACT__
/**
 * p3d_GetMinDistCost
 * Get the cost of a current configuration based 
 * on its minimal distance to the obstacles. 
 * This function is used for algorithm based on configuration
 * spaces with cost functions
 * @param[In] robotPt: the robot
 * @return: the value of the cost for the current 
 * configuration of the robot 
 * Note: For efficiency reasons we suppose that the robot is not
 * in collision (configurations are node configurations), but it 
 * could change. 
 */
extern p3d_vector3 vectMinDist[2];
extern double p3d_GetMinDistCost(p3d_rob* robotPt);
extern double p3d_GetAverageDistCost(p3d_rob* robotPt);
extern void set_kcd_which_test ( p3d_type_col_choice val );
extern p3d_type_col_choice get_kcd_which_test ( void );
extern void set_collision_by_object ( int boolval );
extern int get_collision_by_object ( void );
extern void set_return_kcd_distance_estimate ( int boolval );
extern int get_return_kcd_distance_estimate ( void );
extern int p3d_col_object_is_pure_graphic ( p3d_obj *obj );
extern void p3d_col_activate_pair_of_objects ( p3d_obj *obj1, p3d_obj *obj2 );
extern void p3d_col_activate_object_to_env ( p3d_obj *obj );
extern void p3d_col_activate_pair ( p3d_poly *obj1, p3d_poly *obj2 );
extern void p3d_col_activate_body_obj ( p3d_obj *bod, p3d_obj *obst );
extern void p3d_col_activate_full ( p3d_poly *obj );
extern void p3d_col_activate_all ( void );
extern void p3d_col_deactivate_pair_of_objects ( p3d_obj *obj1, p3d_obj *obj2 );
extern void p3d_col_deactivate_object_to_env ( p3d_obj *obj );
extern void p3d_col_deactivate_pair ( p3d_poly *obj1, p3d_poly *obj2 );
extern void p3d_col_deactivate_full ( p3d_poly *obj );
extern void p3d_col_cur_deactivate_all ( void );
extern void p3d_col_deactivate_rudely_all ( void );
extern int p3d_switch_to_cube ( p3d_poly *p );
extern int p3d_switch_to_DBB ( p3d_poly *p );
extern int p3d_switch_all_to_cube ( void );
extern int p3d_switch_all_to_DBB ( void );
extern int p3d_enable_nbody ( void );
extern int p3d_disable_nbody ( void );
extern int p3d_col_set_user_defined_small_volume ( double val );
extern int p3d_col_get_user_defined_small_volume ( double *val );
extern int p3d_col_set_tolerance ( double value );
extern int p3d_col_get_tolerance ( double *val );
extern void p3d_col_get_dmax ( double *val );
extern void p3d_col_set_dmax ( double val );
extern void p3d_col_set_relative_error ( double epsilon );
extern void p3d_col_get_relative_error ( double *epsilon );
extern int p3d_col_test ( void );
extern int p3d_col_test_all ( void );
extern int p3d_col_test_choice ( void );
extern int p3d_col_test_self_collision ( p3d_rob *robotPt, int with_report );
extern int p3d_col_test_robot_other ( p3d_rob *robotPt1, p3d_rob *robotPt2, int with_report );
extern int p3d_col_test_robot ( p3d_rob *robotPt, int with_report );
extern int p3d_col_test_robot_statics ( p3d_rob *robotPt, int with_report );
extern int p3d_col_report_distance_bodies_obst ( p3d_rob *robotPt, double *distances );
extern int p3d_col_report_distance ( p3d_rob *robotPt, double *distances );
extern int p3d_col_report_closest_points ( p3d_rob *robotPt, p3d_vector3 *points_bodies, p3d_vector3 *points_other, double *distances );
extern int p3d_col_test_pair ( p3d_poly *obj1, p3d_poly *obj2 );
extern int p3d_col_test_act ( void );
extern int p3d_col_number ( void );
extern void p3d_col_get ( int ind, int *id1, int *id2 );
extern void p3d_col_get_report ( int ind, p3d_poly **p1, p3d_poly **p2 );
extern void p3d_col_get_report_obj ( p3d_obj **o1, p3d_obj **o2 );
extern void p3d_col_init_coll ( void );
extern void p3d_col_del_poly ( p3d_poly *p );
extern void p3d_col_set_pos_of_object ( p3d_obj *p, p3d_matrix4 mat );
extern void p3d_col_set_pos ( p3d_poly *p, p3d_matrix4 mat );
extern int p3d_col_test_poly_id ( p3d_poly *p, int id1, int id2 );
extern int p3d_col_test_object_id ( p3d_obj *p, int id1, int id2 );
extern int p3d_col_does_robot_collide ( int rob_nr, int numcoll );
extern void *p3d_col_get_col_BB_poly_fct ( void );
extern void p3d_col_start_current ( void );
extern void p3d_col_start_last ( void );
extern void p3d_col_start ( int c );
extern void p3d_col_stop ( void );
extern void p3d_col_stop_all ( void );
extern void p3d_col_BB_set_mode_col ( void );
extern int p3d_col_get_mode ( void );
extern void p3d_col_set_mode ( int col_mode );
extern int p3d_col_report_mo_closest_points ( p3d_obj *mobjPt, p3d_vector3 point_mo, p3d_vector3 point_o, double *distance );
extern void p3d_col_deactivate_obstacle ( p3d_obj *obstaclePt );
extern void p3d_col_activate_obstacle ( p3d_obj *obstaclePt );
extern int p3d_col_add_obstacle ( p3d_obj *obst );
extern int p3d_col_add_obstacle_list ( p3d_obj **obst, int nof_obst );

extern void p3d_col_deactivate_robot(p3d_rob *robot);
extern void p3d_col_activate_robot(p3d_rob *robot);
extern void p3d_col_deactivate_robot_robot(p3d_rob *robot1, p3d_rob *robot2);
extern void p3d_col_activate_robot_robot(p3d_rob *robot1, p3d_rob *robot2);

extern int p3d_col_test_pair(p3d_obj *obj1, p3d_obj *obj2);
extern int p3d_col_test_robot_obj(p3d_rob *robot, p3d_obj *ob);
extern int p3d_col_deactivate_obj(p3d_obj *ob);
extern int p3d_col_activate_obj(p3d_obj *ob);
extern double p3d_distanceObjToObj(p3d_obj* o1, p3d_obj* o2);

extern double p3d_col_obj_obj_distance(p3d_obj *obj1, p3d_obj *obj2, p3d_vector3 closest_point_obj1, p3d_vector3 closest_point_obj2);
extern double p3d_col_robot_robot_distance(p3d_rob *robot1, p3d_rob *robot2, p3d_vector3 closest_point_rob1, p3d_vector3 closest_point_rob2);
extern double p3d_col_robot_environment_distance(p3d_rob *robot, p3d_vector3 closest_point_rob, p3d_vector3 closest_point_obst);
extern double p3d_col_robot_robot_weighted_distance(p3d_rob *robot1, p3d_rob *robot2);
extern double p3d_distanceObjToObj(p3d_obj* o1, p3d_obj* o2);
extern double p3d_distanceObjToEnv(p3d_obj* o);
extern double p3d_obj_point_distance(p3d_obj *o, p3d_vector3 point);
extern int p3d_col_rob_sphere(p3d_rob *rrobotPt, p3d_vector3 center, double radius);
extern int p3d_col_rob_collision_cloud(p3d_rob *robotPt);
extern void p3d_print_col_pair();
#endif /* __CEXTRACT__ */
