SET(BM3D_MODULE_NAME p3d)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(
p3d_BB.c 
p3d_autocol.c 
p3d_config.c 
p3d_del.c 
p3d_env.c 
p3d_get.c 
p3d_halton.c 
p3d_human_arm_ik.c 
p3d_ik.c 
p3d_ik_kuka.c
p3d_info.c 
p3d_jacobian.c 
p3d_jnt_base.c 
p3d_jnt_fixed.c 
p3d_jnt_freeflyer.c 
p3d_jnt_knee.c 
p3d_jnt_plan.c 
p3d_jnt_rotate.c 
p3d_jnt_translate.c 
p3d_joints.c 
p3d_matrix.c 
p3d_parallel.c 
p3d_poly.c 
p3d_prim.c 
p3d_random.c 
p3d_rlg.c 
p3d_copy_robot.cpp 
p3d_rw_env.c 
p3d_rw_jnt.c 
p3d_rw_scenario.c 
p3d_rw_traj.c 
p3d_rwXmlBasics.c 
p3d_rwXmlTraj.c 
p3d_rw_util.c 
p3d_set.c 
p3d_setpos.c 
polyhedre.c 
env.cpp
MTRand.cpp
ParametersEnv.cpp
)


IF(P3D_COLLISION_CHECKING)
BM3D_SRC_SUBDIR_PROCESS(
p3d_constraints.c 
)
ENDIF(P3D_COLLISION_CHECKING)

IF(USE_GBM)
BM3D_SRC_SUBDIR_PROCESS(
	 p3d_ik_pa10.c 
	 p3d_ik_lwr.c	       
   p3d_ik_pr2.c	      
)
ENDIF(USE_GBM)

IF(USE_COLLADA15DOM)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/urdf_interface/tinyxml)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/urdf_interface)
BM3D_SRC_SUBDIR_PROCESS(
	collada_parser.h
	collada_parser.cpp
	./urdf_interface/joint.cpp
	./urdf_interface/link.cpp
	./urdf_interface/tinyxml/tinystr.cpp
	./urdf_interface/tinyxml/tinyxml.cpp
	./urdf_interface/tinyxml/tinyxmlparser.cpp
	./urdf_interface/tinyxml/tinyxmlerror.cpp
 	urdf_p3d_converter.cpp
 	p3d_load_collada.cpp
)
ENDIF(USE_COLLADA15DOM)

BM3D_QT_GENERATE_MOC(
ParametersEnv.hpp
env.hpp
)