#ifndef GP_GRASPING_UTILS_PROTO_H
#define GP_GRASPING_UTILS_PROTO_H

extern std::string gpHand_type_to_string(gpHand_type hand_type);

extern std::string gpHand_type_to_folder_name(gpHand_type hand_type);

extern std::string gpHand_suffix_from_ID(int id);

extern int gpGet_arm_base_frame(p3d_rob *robot, p3d_matrix4 frame );

extern int gpGet_platform_frame(p3d_rob *robot, p3d_matrix4 frame);

extern int gpGet_wrist_frame(p3d_rob *robot, p3d_matrix4 frame );

extern int gpGet_wrist_frame(p3d_rob *robot, int armID, p3d_matrix4 frame );

extern int gpHand_frame_from_grasp_frame(p3d_matrix4 grasp_frame, p3d_matrix4 hand_frame, gpHand_properties &hand_properties);

extern int gpGrasp_frame_from_hand_frame(p3d_matrix4 hand_frame, p3d_matrix4 grasp_frame, gpHand_properties &hand_properties);

extern int gpGrasp_frame_from_end_effector_frame(p3d_matrix4 end_effector_frame, p3d_matrix4 grasp_frame, gpHand_properties &hand_properties);

extern void gpDraw_friction_cone ( p3d_vector3 c, p3d_vector3 normal, double mu, int nb_slices, double length );

extern void gpDraw_friction_cone2(p3d_vector3 c, p3d_vector3 normal, double mu, int nb_slices, double length);

extern configPt gpRandom_robot_base(p3d_rob *robot, double innerRadius, double outerRadius, p3d_vector3 objLoc, gpArm_type arm_type);

extern int gpGet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand_properties, double q[4], int finger_index, int handID= 0);

extern int gpSet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand_properties, double q[4], int finger_index, int handID= 0);

extern int gpSAHfinger_forward_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, double q[4], p3d_vector3 p, p3d_vector3 fingerpad_normal, int finger_index);

extern int gpSAHfinger_inverse_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, p3d_vector3 p, double q[4], p3d_vector3 fingerpad_normal, int finger_index);

extern int gpOpen_hand(p3d_rob *robot, gpHand_properties &hand);

extern int gpClose_hand(p3d_rob *robot, gpHand_properties &hand);

extern int gpGet_platform_configuration(p3d_rob *robot, double &x, double &y, double &theta);

extern int gpSet_platform_configuration(p3d_rob *robot, double x, double y, double theta);
 
extern int gpGet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double &q1, double &q2, double &q3, double &q4, double &q5, double &q6);

extern int gpSet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double q1, double q2, double q3, double q4, double q5, double q6, bool verbose= false);

extern int gpSet_grasp_configuration(p3d_rob *robot, gpHand_properties &hand, const gpGrasp &grasp, int handID= 0);

extern int gpSet_grasp_open_configuration(p3d_rob *robot, gpHand_properties &hand, const gpGrasp &grasp, int handID= 0);

extern int gpGet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, int handID, std::vector<double> &config);

extern int gpSet_hand_configuration(p3d_rob *robot, gpHand_properties &handProp, std::vector<double> config, bool verbose,  int handID= 0);

extern int gpFold_arm(p3d_rob *robot, gpArm_type arm_type);

extern int gpDeactivate_arm_collisions(p3d_rob *robot, int armID= 0);

extern int gpActivate_arm_collisions(p3d_rob *robot, int armID= 0);

extern int gpDeactivate_hand_collisions(p3d_rob *robot, int handID= 0);

extern int gpActivate_hand_collisions(p3d_rob *robot, int handID= 0);

extern int gpDeactivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand);

extern int gpActivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand);

extern int gpDeactivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand, int handID= 0);

extern int gpActivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand, int handID= 0);

extern int gpSample_obj_surface(p3d_obj *object, double step, double shift, std::list<gpContact> &contactList);

#endif
