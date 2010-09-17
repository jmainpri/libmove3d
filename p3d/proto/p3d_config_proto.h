/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__
/**
 * Compute the square distance between two configurations using
 * only the active dofs
 * 
 * Input:  The robot,
 *         the two configurations
 */
extern double p3d_ActiveDistConfig(p3d_rob * robotPt, configPt q_i, configPt q_f);
extern configPt p3d_alloc_config_n ( int nb_dof );
extern configPt p3d_alloc_config ( p3d_rob *robotPt );
extern configPt p3d_alloc_user_config ( p3d_rob *robotPt );
extern configPt p3d_alloc_body_config ( void );
extern void p3d_destroy_body_config ( configPt q );
extern configPt p3d_copy_config_n ( int nb_dof, configPt q );
extern configPt p3d_copy_config ( p3d_rob * robotPt, configPt q );
extern configPt p3d_copy_user_config_to_config ( p3d_rob *robotPt, configPt q_user );
extern configPt p3d_copy_user_config_to_user_config ( p3d_rob *robotPt, configPt q_user );
extern configPt p3d_copy_config_to_user_config ( p3d_rob *robotPt, configPt q );
extern void p3d_copy_config_n_into ( int nb_dof, configPt config1, configPt *config2Pt );
extern void p3d_copy_config_into ( p3d_rob *robotPt, configPt config1, configPt *config2Pt );
extern void p3d_copy_user_config_into_config ( p3d_rob *robotPt, configPt user_config1, configPt * config2 );
extern void p3d_copy_user_config_into_user_config ( p3d_rob *robotPt, configPt user_config1, configPt * user_config2 );
extern void p3d_copy_config_into_user_config ( p3d_rob *robotPt, configPt config1, configPt * user_config2 );
extern configPt p3d_copy_config_rad_to_deg ( p3d_rob *robotPt, configPt q );
extern configPt p3d_copy_config_deg_to_rad ( p3d_rob *robotPt, configPt q );
extern void p3d_addConfig ( p3d_rob *rob, configPt q1, configPt q2, configPt q );
extern void p3d_addConfig2PI ( p3d_rob *rob, configPt q1, configPt q2, configPt q );
extern void p3d_subConfig ( p3d_rob *rob, configPt q1, configPt q2, configPt dq );
extern int p3d_equal_config_n ( int nb_dof, configPt q_i, configPt q_f );
extern int p3d_equal_config ( p3d_rob *robotPt, configPt q_i, configPt q_f );

extern int p3d_equal_config_n_offset(int nb_dof, int offset, configPt q_i, configPt q_f);

extern double p3d_dist_config ( p3d_rob * robotPt, configPt q_i, configPt q_f );
extern double p3d_dist_config_2 ( p3d_rob * robotPt, configPt q_i, configPt q_f );
extern void p3d_convert_config_deg_to_rad ( p3d_rob *robotPt, configPt q_deg, configPt *q_rad );
extern void p3d_convert_config_rad_to_deg ( p3d_rob *robotPt, configPt q_rad, configPt *q_deg );
extern void p3d_destroy_config_n ( int nb_dof, configPt cfg );
extern void p3d_destroy_config ( p3d_rob *robotPt, configPt cfg );
extern void p3d_destroy_user_config ( p3d_rob *robotPt, configPt cfg );
extern double p3d_point_dist ( p3d_point p1, p3d_point p2 );
extern void p3d_set_robot_in_joint_limits ( p3d_rob *robotPt, configPt q );
extern void print_config ( p3d_rob *robotPt, configPt q );
extern void print_config_one_line_degrees(p3d_rob *robotPt, configPt q);
extern void fprint_config_one_line ( FILE *file, p3d_rob *robotPt, configPt q );
extern void p3d_middleConfig(p3d_rob *rob, configPt q1, configPt q2, configPt q);
extern void p3dCopyPassive(p3d_rob*robotPt, configPt qSource, configPt qGoal);
extern int p3d_isNullConfig(p3d_rob* robot, configPt q);
//start path deform
double p3d_stay_within_sphere(p3d_rob* robotPt, double *distances);
//end path deform
#endif /* __CEXTRACT__ */
