
#ifndef GP_GRASP_GENERATION_PROTO_H
#define GP_GRASP_GENERATION_PROTO_H

extern p3d_matrix4 *gpSample_grasp_frames(p3d_vector3 cmass, p3d_matrix3 iaxes, double iaabb[6], double translationStep, unsigned int nbDirections, double rotationStep, unsigned int *nbSamples);

extern int gpSample_grasp_frames2(p3d_polyhedre *polyhedron, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, unsigned int nbFramesMax, std::vector<gpHTMatrix> &frames);

extern int gpGrasps_from_grasp_frame(p3d_rob *robot, p3d_rob *object, int body_index, p3d_matrix4 gFrame, gpHand_properties &handProp, std::list<class gpGrasp> &graspList);

extern int gpGrasps_from_grasp_frame_gripper(p3d_polyhedre *polyhedron, p3d_matrix4 gFrame, gpHand_properties &handProp, std::list<class gpGrasp> &graspList);

extern int gpGrasps_from_grasp_frame_SAHand(p3d_rob *robot, p3d_rob *object, int body_index, p3d_matrix4 gFrame, gpHand_properties &handProp, std::list<class gpGrasp> &graspList);

extern int gpGrasps_from_grasp_frame_SAHand2(p3d_rob *robot, p3d_rob *object, int body_index, p3d_matrix4 gFrame, gpHand_properties &handProp, gpKdTree &kdtree, std::list<class gpGrasp> &graspList);

extern int gpCompute_grasp_open_config(p3d_rob *robot, p3d_rob *object, int body_index, gpGrasp &grasp);

extern int gpGrasp_frame_from_inertia_axes ( p3d_matrix3 iaxes, p3d_vector3 cmass, int direction, double displacement, int axis, double angle, p3d_matrix4 gframe );

extern int gpGrasp_generation(p3d_rob *robot, p3d_rob *object, int body_index, gpHand_properties &handProp, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, std::list<class gpGrasp> &graspList);

extern int gpInverse_geometric_model_freeflying_hand(p3d_rob *robot, p3d_matrix4 objectFrame, p3d_matrix4 graspFrame, gpHand_properties &handProp, configPt q);

extern int gpForward_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, bool display);

extern int gpInverse_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt q);

extern int gpInverse_geometric_model(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt q);

extern int gpGrasp_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, int body_index, gpHand_properties &handProp);

extern int gpGrasp_compute_open_configs(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp);

extern int gpGrasp_context_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp);

extern int gpGrasp_stability_filter(std::list<gpGrasp> &graspList);

extern configPt gpFind_grasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &handProp);

extern int gpFind_grasp_and_pregrasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &handProp, double distance, configPt qpregrasp, configPt qgrasp);

extern int gpGet_grasp_list_SAHand(std::string object_to_grasp, int hand_to_use, std::list<gpGrasp> &graspList);

extern int gpDouble_grasp_generation(p3d_rob *robot1, p3d_rob *robot2, p3d_rob *object, std::list<class gpGrasp> &graspList1, std::list<class gpGrasp> &graspList2, std::list<class gpDoubleGrasp> &doubleGraspList);

#endif
