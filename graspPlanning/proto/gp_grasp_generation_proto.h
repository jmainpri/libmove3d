/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */

#ifndef GP_GRASP_GENERATION_PROTO_H
#define GP_GRASP_GENERATION_PROTO_H

extern int gpSample_grasp_frames(p3d_polyhedre *polyhedron, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, unsigned int nbFramesMax, std::vector<gpHTMatrix> &frames);

extern int gpGrasps_from_grasp_frame_gripper(p3d_polyhedre *polyhedron, p3d_matrix4 gFrame, gpHand_properties &handProp, std::list<class gpGrasp> &graspList);

extern int gpGrasp_generation_pr2_gripper(p3d_polyhedre *polyhedron, gpHand_properties &handProp, double mu, std::list<class gpGrasp> &graspList);

extern int gpGrasps_from_grasp_frame_SAHand(p3d_rob *robot, p3d_rob *object, p3d_matrix4 gFrame, gpHand_properties &handProp, gpKdTree &kdtree, std::list<class gpGrasp> &graspList);

extern int gpGrasp_frame_from_inertia_axes(p3d_matrix3 iaxes, p3d_vector3 cmass, int direction, double displacement, int axis, double angle, p3d_matrix4 gframe);

extern int gpGrasp_generation(p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, std::list<class gpGrasp> &graspList);

extern int gpInverse_geometric_model_freeflying_hand(p3d_rob *robot, p3d_matrix4 objectFrame, p3d_matrix4 graspFrame, gpHand_properties &handProp, configPt q);

extern int gpForward_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, bool display);

extern int gpInverse_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpInverse_geometric_model_and_collision_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpInverse_geometric_model_LWR(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpInverse_geometric_model_and_collision_LWR(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpInverse_geometric_model_arm(p3d_rob *robot, gpArm_type arm_type, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpInverse_geometric_model_and_collision_arm(p3d_rob *robot, gpArm_type arm_type, p3d_matrix4 Tend_eff, configPt cfg);

extern int gpGrasp_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp);

extern int gpCompute_grasp_open_configs(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object);

extern int gpCompute_grasp_open_config(p3d_rob *robot, class gpDoubleGrasp &doubleGrasp, p3d_rob *object, int hand_to_open);

extern int gpGrasp_context_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp);

extern int gpGrasp_stability_filter(std::list<gpGrasp> &graspList);

extern int gpCompute_grasps_best_placement(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &hand);

extern int gpGrasp_visibility_filter(p3d_rob *robot, p3d_rob *object, p3d_jnt *cam_jnt, double camera_fov, int imageWidth, int imageHeight, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpHand_properties &hand);

extern configPt gpFind_grasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &handProp);

extern int gpFind_grasp_and_pregrasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &handProp, double distance, configPt qpregrasp, configPt qgrasp);

extern int gpGet_grasp_list(const std::string &object_to_grasp, gpHand_type hand_type, std::list<gpGrasp> &graspList);

extern int gpExpand_grasp_list(p3d_rob *robot, std::list<gpGrasp> &graspList, int nbTries);

extern int gpGrasp_handover_filter(p3d_rob *robot1, p3d_rob *robot2, p3d_rob *object, std::list<class gpGrasp> &graspList1, const std::list<class gpGrasp> &graspList2);

extern int gpDouble_grasp_generation(p3d_rob *robot1, p3d_rob *robot2, p3d_rob *object, const std::list<class gpGrasp> &graspList1, const std::list<class gpGrasp> &graspList2, std::list<class gpDoubleGrasp> &doubleGraspList);

extern int gpReduce_grasp_list_size(const std::list<gpGrasp> &originalList, std::list<gpGrasp> &reducedList, unsigned int maxSize);

extern int gpRemove_edge_contacts(std::list<gpGrasp> &graspList, double angle, double step);

extern int gpCompute_grasp_qualities_and_sort(std::list<gpGrasp> &graspList);

#endif
