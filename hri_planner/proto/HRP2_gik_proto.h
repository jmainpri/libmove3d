
#ifndef __CEXTRACT__

extern int M3D_GIK_TEST ( int hand_to_reach);
extern int show_gik_sol( void );
extern int HRP2_hand_reach(p3d_vector3 target_in_global_frame, int hand_by_reach, double task_duration, int state, int thumb_up_constraint, int use_body_part);
extern double* get_HRP2_hand_center_in_global_frame(int for_hand);//1 for left, 2 for right hand
extern int create_HRP2_robot(int state);

extern int Hand_Clench ( int for_hand, double clench );
extern int show_3d_grid_Bounding_box_for_HRP2_GIK();
extern int create_HRP2_robot_for_GIK_in_Move3d(int state);
extern int HRP2_look_at(p3d_vector3 look_at_point_in_global_frame, double task_duration, int state, int use_body_part);
extern int hrp2_to_M3D_ConfigPt( ghrp2_config_t *cfg, configPt m3dconfig);
extern int HRP2_get_joint_mask(int hand_by_reach, int state, int use_body_part); // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body
extern int push_HRP2_hand_pos_constraint(p3d_vector3 target_in_global_frame, int hand_by_reach); // hand_by_reach =1 for left hand, 2 for right hand
extern int get_HRP2_GIK_sol();
extern int create_HRP2_hand_pos_constraint(int hand_by_reach);
extern int delete_HRP2_hand_pos_constraint();
extern int initialize_constraints_stack_for_standing_HRP2();
extern int push_HRP2_look_at_constraint(p3d_vector3 look_at_point_in_global_frame);
extern int delete_HRP2_look_at_constraint();
extern int get_parallel_constraint_interpolation(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach, double task_duration, double start_time, double sampling_period, int priority);
extern int create_HRP2_hand_parallel_constraint(int hand_by_reach);
extern int delete_HRP2_hand_parallel_constraint();
extern int push_parallel_constraint_interpolated_element(double at_time);
extern int get_HRP2_look_at_constraint_interpolation(p3d_vector3 look_at_point_in_global_frame,  double task_duration, double start_time, double sampling_period, int priority);
extern int push_HRP2_look_at_constraint_interpolated_element(double at_time);
extern int create_HRP2_look_at_constraint();
extern int push_HRP2_hand_parallel_constraint(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach);
#endif /* __CEXTRACT__ */
